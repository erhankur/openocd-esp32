// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <string.h>

#include <helper/log.h>
#include <jtag/adapter.h>
#include "libusb_helper.h"

/*
 * comment from libusb:
 * As per the USB 3.0 specs, the current maximum limit for the depth is 7.
 */
#define MAX_USB_PORTS	7

static struct libusb_context *jtag_libusb_context; /**< Libusb context **/
static struct libusb_device **devs; /**< The usb device list **/

static int jtag_libusb_error(int err)
{
	switch (err) {
	case LIBUSB_SUCCESS:
		return ERROR_OK;
	case LIBUSB_ERROR_TIMEOUT:
		return ERROR_TIMEOUT_REACHED;
	case LIBUSB_ERROR_IO:
	case LIBUSB_ERROR_INVALID_PARAM:
	case LIBUSB_ERROR_ACCESS:
	case LIBUSB_ERROR_NO_DEVICE:
	case LIBUSB_ERROR_NOT_FOUND:
	case LIBUSB_ERROR_BUSY:
	case LIBUSB_ERROR_OVERFLOW:
	case LIBUSB_ERROR_PIPE:
	case LIBUSB_ERROR_INTERRUPTED:
	case LIBUSB_ERROR_NO_MEM:
	case LIBUSB_ERROR_NOT_SUPPORTED:
	case LIBUSB_ERROR_OTHER:
		return ERROR_FAIL;
	default:
		return ERROR_FAIL;
	}
}

bool jtag_libusb_match_ids(struct libusb_device_descriptor *dev_desc,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned int i = 0; vids[i]; i++) {
		if (dev_desc->idVendor == vids[i] &&
			dev_desc->idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
static bool jtag_libusb_location_equal(struct libusb_device *device)
{
	uint8_t port_path[MAX_USB_PORTS];
	uint8_t dev_bus;
	int path_len;

	path_len = libusb_get_port_numbers(device, port_path, MAX_USB_PORTS);
	if (path_len == LIBUSB_ERROR_OVERFLOW) {
		LOG_WARNING("cannot determine path to usb device! (more than %i ports in path)\n",
			MAX_USB_PORTS);
		return false;
	}
	dev_bus = libusb_get_bus_number(device);

	return adapter_usb_location_equal(dev_bus, port_path, path_len);
}
#else /* HAVE_LIBUSB_GET_PORT_NUMBERS */
static bool jtag_libusb_location_equal(struct libusb_device *device)
{
	return true;
}
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */


/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(struct libusb_device_handle *device, uint8_t str_index,
									const char *string)
{
	int retval;
	bool matched;
	char desc_string[256+1]; /* Max size of string descriptor */

	if (str_index == 0)
		return false;

	retval = libusb_get_string_descriptor_ascii(device, str_index,
			(unsigned char *)desc_string, sizeof(desc_string)-1);
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor_ascii() failed with '%s'", libusb_error_name(retval));
		return false;
	}

	/* Null terminate descriptor string in case it needs to be logged. */
	desc_string[sizeof(desc_string)-1] = '\0';

	matched = strncmp(string, desc_string, sizeof(desc_string)) == 0;
	if (!matched)
		LOG_DEBUG("Device serial number '%s' doesn't match requested serial '%s'",
			desc_string, string);
	return matched;
}

static bool jtag_libusb_match_serial(struct libusb_device_handle *device,
		struct libusb_device_descriptor *dev_desc, const char *serial,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial)
{
	if (string_descriptor_equal(device, dev_desc->iSerialNumber, serial))
		return true;

	/* check the alternate serial helper */
	if (!adapter_get_alternate_serial)
		return false;

	/* get the alternate serial */
	char *alternate_serial = adapter_get_alternate_serial(device, dev_desc);

	/* check possible failures */
	if (!alternate_serial)
		return false;

	/* then compare and free the alternate serial */
	bool match = false;
	if (strcmp(serial, alternate_serial) == 0)
		match = true;
	else
		LOG_DEBUG("Device alternate serial number '%s' doesn't match requested serial '%s'",
				alternate_serial, serial);

	free(alternate_serial);
	return match;
}

static int jtag_libusb_get_dev_location(struct libusb_device *dev, char *loc, int loc_len)
{
	int k, len, wr;
	uint8_t dev_bus = 0;
	uint8_t port_path[MAX_USB_PORTS];
	int path_len = 0;

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
	path_len = libusb_get_port_numbers(dev, port_path, MAX_USB_PORTS);
	if (path_len == LIBUSB_ERROR_OVERFLOW) {
		LOG_WARNING("cannot determine path to usb device! (more than %i ports in path)", MAX_USB_PORTS);
		return ERROR_FAIL;
	}
	dev_bus = libusb_get_bus_number(dev);
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */

	len = snprintf(loc, loc_len, "%d", dev_bus);
	if (len < 0 || len >= (loc_len - len)) {
		*loc = 0;
		return ERROR_FAIL;
	}

	for (k = 0; k < path_len; k++) {
		wr = snprintf(&loc[len], loc_len - len, k == 0 ? "-%d" : ".%d", port_path[k]);
		if (wr < 0 || wr >= (loc_len - len)) {
			*loc = 0;
			return ERROR_FAIL;
		}
		len += wr;
	}
	return ERROR_OK;
}

int jtag_libusb_get_dev_location_by_handle(struct libusb_device_handle *dev, char *loc, int loc_len)
{
	return jtag_libusb_get_dev_location(libusb_get_device(dev), loc, loc_len);
}

int jtag_libusb_get_devs_locations(const uint16_t vids[], const uint16_t pids[], char ***locations)
{
	int cnt, idx, devs_cnt = 0;
	struct libusb_device **list;
	struct libusb_device_descriptor desc;
	char **locs;
	/* <bus>-<port>[.<port>]... */
	#define MAX_DEV_LOCATION_LEN	128

	cnt = libusb_get_device_list(jtag_libusb_context, &list);
	if (cnt <= 0) {
		LOG_WARNING("Cannot get devices list (%d)!", cnt);
		return 0;
	}

	locs = calloc(cnt, sizeof(char *));
	if (!locs) {
		LOG_ERROR("Unable to allocate space USB devices list!");
		libusb_free_device_list(list, 1);
		return 0;
	}
	for (idx = 0; idx < cnt; idx++) {
		if (libusb_get_device_descriptor(list[idx], &desc) != 0)
			continue;
		if (!jtag_libusb_match_ids(&desc, vids, pids))
			continue;

		locs[devs_cnt] = calloc(1, MAX_DEV_LOCATION_LEN);
		if (!locs[devs_cnt]) {
			LOG_ERROR("Unable to allocate space USB device location!");
			jtag_libusb_free_devs_locations(locs, devs_cnt);
			libusb_free_device_list(list, true);
			return 0;
		}
		if (jtag_libusb_get_dev_location(list[idx], locs[devs_cnt], MAX_DEV_LOCATION_LEN) != ERROR_OK)
			LOG_WARNING("Cannot get location for usb device!");

		devs_cnt++;
	}
	*locations = locs;

	libusb_free_device_list(list, true);

	return devs_cnt;
}

void jtag_libusb_free_devs_locations(char *locations[], int cnt)
{
	int i;

	if (!locations || cnt == 0)
		return;

	for (i = 0; i < cnt; i++) {
		if (locations[i])
			free(locations[i]);
	}
	free(locations);
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *product, struct libusb_device_handle **out,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial)
{
	int cnt, idx, err_code;
	int retval = ERROR_FAIL;
	bool serial_mismatch = false;
	bool product_mismatch = false;
	struct libusb_device_handle *libusb_handle = NULL;
	const char *serial = adapter_get_required_serial();

	if (libusb_init(&jtag_libusb_context) < 0)
		return ERROR_FAIL;

	cnt = libusb_get_device_list(jtag_libusb_context, &devs);

	for (idx = 0; idx < cnt; idx++) {
		struct libusb_device_descriptor dev_desc;

		if (libusb_get_device_descriptor(devs[idx], &dev_desc) != 0)
			continue;

		if (!jtag_libusb_match_ids(&dev_desc, vids, pids))
			continue;

		if (adapter_usb_get_location() && !jtag_libusb_location_equal(devs[idx]))
			continue;

		err_code = libusb_open(devs[idx], &libusb_handle);

		if (err_code) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(err_code));
			continue;
		}

		/* Device must be open to use libusb_get_string_descriptor_ascii. */
		if (serial &&
				!jtag_libusb_match_serial(libusb_handle, &dev_desc, serial, adapter_get_alternate_serial)) {
			serial_mismatch = true;
			libusb_close(libusb_handle);
			continue;
		}

		if (product &&
				!string_descriptor_equal(libusb_handle, dev_desc.iProduct, product)) {
			product_mismatch = true;
			libusb_close(libusb_handle);
			continue;
		}

		/* Success. */
		*out = libusb_handle;
		retval = ERROR_OK;
		serial_mismatch = false;
		product_mismatch = false;
		break;
	}
	if (cnt >= 0)
		libusb_free_device_list(devs, 1);

	if (serial_mismatch)
		LOG_INFO("No device matches the serial string");

	if (product_mismatch)
		LOG_INFO("No device matches the product string");

	if (retval != ERROR_OK)
		libusb_exit(jtag_libusb_context);

	return retval;
}

void jtag_libusb_close(struct libusb_device_handle *dev)
{
	/* Close device */
	libusb_close(dev);

	libusb_exit(jtag_libusb_context);
}

int jtag_libusb_control_transfer(struct libusb_device_handle *dev, uint8_t request_type,
		uint8_t request, uint16_t value, uint16_t index, char *bytes,
		uint16_t size, unsigned int timeout, int *transferred)
{
	int retval = libusb_control_transfer(dev, request_type, request, value, index,
				(unsigned char *)bytes, size, timeout);

	if (retval < 0) {
		LOG_ERROR("libusb_control_transfer error: %s", libusb_error_name(retval));
		if (transferred)
			*transferred = 0;
		return jtag_libusb_error(retval);
	}

	if (transferred)
		*transferred = retval;

	return ERROR_OK;
}

int jtag_libusb_bulk_write(struct libusb_device_handle *dev, int ep, char *bytes,
			   int size, int timeout, int *transferred)
{
	int ret;

	*transferred = 0;

	ret = libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				   transferred, timeout);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_bulk_write error: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	return ERROR_OK;
}

int jtag_libusb_bulk_read(struct libusb_device_handle *dev, int ep, char *bytes,
			  int size, int timeout, int *transferred)
{
	int ret;

	*transferred = 0;

	ret = libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
				   transferred, timeout);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_bulk_read error: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	return ERROR_OK;
}

int jtag_libusb_set_configuration(struct libusb_device_handle *devh,
		int configuration)
{
	struct libusb_device *udev = libusb_get_device(devh);
	int retval = -99;

	struct libusb_config_descriptor *config = NULL;
	int current_config = -1;

	retval = libusb_get_configuration(devh, &current_config);
	if (retval != 0)
		return retval;

	retval = libusb_get_config_descriptor(udev, configuration, &config);
	if (retval != 0 || !config)
		return retval;

	/* Only change the configuration if it is not already set to the
	   same one. Otherwise this issues a lightweight reset and hangs
	   LPC-Link2 with JLink firmware. */
	if (current_config != config->bConfigurationValue)
		retval = libusb_set_configuration(devh, config->bConfigurationValue);

	libusb_free_config_descriptor(config);

	return retval;
}

int jtag_libusb_choose_interface(struct libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type)
{
	struct libusb_device *udev = libusb_get_device(devh);
	const struct libusb_interface *inter;
	const struct libusb_interface_descriptor *interdesc;
	const struct libusb_endpoint_descriptor *epdesc;
	struct libusb_config_descriptor *config;

	*usb_read_ep = *usb_write_ep = 0;

	libusb_get_config_descriptor(udev, 0, &config);
	for (int i = 0; i < (int)config->bNumInterfaces; i++) {
		inter = &config->interface[i];

		interdesc = &inter->altsetting[0];
		for (int k = 0;
		     k < (int)interdesc->bNumEndpoints; k++) {
			if ((bclass > 0 && interdesc->bInterfaceClass != bclass) ||
			    (subclass > 0 && interdesc->bInterfaceSubClass != subclass) ||
			    (protocol > 0 && interdesc->bInterfaceProtocol != protocol))
				continue;

			epdesc = &interdesc->endpoint[k];
			if (trans_type > 0 && (epdesc->bmAttributes & 0x3) != trans_type)
				continue;

			uint8_t epnum = epdesc->bEndpointAddress;
			bool is_input = epnum & 0x80;
			LOG_DEBUG("usb ep %s %02x",
				  is_input ? "in" : "out", epnum);

			if (is_input)
				*usb_read_ep = epnum;
			else
				*usb_write_ep = epnum;

			if (*usb_read_ep && *usb_write_ep) {
				LOG_DEBUG("Claiming interface %d", (int)interdesc->bInterfaceNumber);
				libusb_claim_interface(devh, (int)interdesc->bInterfaceNumber);
				libusb_free_config_descriptor(config);
				return ERROR_OK;
			}
		}
	}
	libusb_free_config_descriptor(config);

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct libusb_device *dev, uint16_t *pid)
{
	struct libusb_device_descriptor dev_desc;

	if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
		*pid = dev_desc.idProduct;

		return ERROR_OK;
	}

	return ERROR_FAIL;
}

int jtag_libusb_get_serial(struct libusb_device_handle *devh, const char **serial)
{
	struct libusb_device *dev = libusb_get_device(devh);
	struct libusb_device_descriptor dev_desc;

	if (serial && libusb_get_device_descriptor(dev, &dev_desc) == 0) {

		if (dev_desc.iSerialNumber == 0)
			return ERROR_FAIL;

		char desc_string[256 + 1]; /* Max size of string descriptor */
		int ret = libusb_get_string_descriptor_ascii(devh, dev_desc.iSerialNumber,
				(unsigned char *)desc_string, sizeof(desc_string) - 1);
		if (ret < 0) {
			LOG_ERROR("libusb_get_string_descriptor_ascii() failed with %d", ret);
			return ERROR_FAIL;
		}
		desc_string[sizeof(desc_string) - 1] = '\0';
		*serial = strdup(desc_string);
		return *serial ? ERROR_OK : ERROR_FAIL;
	}

	return ERROR_FAIL;
}

libusb_device *jtag_libusb_find_device(const uint16_t vids[], const uint16_t pids[], const char *serial)
{
	libusb_device **devices, *found_dev = NULL;

	int cnt = libusb_get_device_list(jtag_libusb_context, &devices);

	for (int idx = 0; idx < cnt; idx++) {
		struct libusb_device_descriptor dev_desc;
		struct libusb_device_handle *libusb_handle = NULL;

		if (libusb_get_device_descriptor(devices[idx], &dev_desc) != 0)
			continue;

		if (!jtag_libusb_match_ids(&dev_desc, vids, pids))
			continue;

		LOG_DEBUG("USB dev found %x:%x @ %d:%d-%d", dev_desc.idVendor, dev_desc.idProduct,
			libusb_get_bus_number(devices[idx]),
			libusb_get_port_number(devices[idx]),
			libusb_get_device_address(devices[idx]));

		if (serial) {
			int ret = libusb_open(devices[idx], &libusb_handle);
			if (ret) {
				LOG_ERROR("libusb_open() failed with %s",
					libusb_error_name(ret));
				continue;
			}
			bool found = string_descriptor_equal(libusb_handle, dev_desc.iSerialNumber, serial);
			libusb_close(libusb_handle);
			if (!found)
				continue;
		}
		found_dev = devices[idx];
		libusb_ref_device(found_dev);
		break;
	}
	if (cnt >= 0)
		libusb_free_device_list(devices, 1);

	return found_dev;
}

int jtag_libusb_handle_events_completed(int *completed)
{
	return libusb_handle_events_completed(jtag_libusb_context, completed);
}

static enum {
	DEV_MEM_NOT_YET_DECIDED,
	DEV_MEM_AVAILABLE,
	DEV_MEM_FALLBACK_MALLOC
} dev_mem_allocation;

/* Older libusb does not implement following API calls - define stubs instead */
#if !defined(LIBUSB_API_VERSION) || (LIBUSB_API_VERSION < 0x01000105)
static uint8_t *libusb_dev_mem_alloc(libusb_device_handle *devh, size_t length)
{
	return NULL;
}

static int libusb_dev_mem_free(libusb_device_handle *devh,
							   uint8_t *buffer, size_t length)
{
	return LIBUSB_ERROR_NOT_SUPPORTED;
}
#endif

uint8_t *oocd_libusb_dev_mem_alloc(libusb_device_handle *devh,
			size_t length)
{
	uint8_t *buffer = NULL;
	if (dev_mem_allocation != DEV_MEM_FALLBACK_MALLOC)
		buffer = libusb_dev_mem_alloc(devh, length);

	if (dev_mem_allocation == DEV_MEM_NOT_YET_DECIDED)
		dev_mem_allocation = buffer ? DEV_MEM_AVAILABLE : DEV_MEM_FALLBACK_MALLOC;

	if (dev_mem_allocation == DEV_MEM_FALLBACK_MALLOC)
		buffer = malloc(length);

	return buffer;
}

int oocd_libusb_dev_mem_free(libusb_device_handle *devh,
		uint8_t *buffer, size_t length)
{
	if (!buffer)
		return ERROR_OK;

	switch (dev_mem_allocation) {
	case DEV_MEM_AVAILABLE:
		return jtag_libusb_error(libusb_dev_mem_free(devh, buffer, length));

	case DEV_MEM_FALLBACK_MALLOC:
		free(buffer);
		return ERROR_OK;

	case DEV_MEM_NOT_YET_DECIDED:
		return ERROR_FAIL;
	}
	return ERROR_FAIL;
}

void libusb_list_devices(void)
{
	struct libusb_device **list;

	int cnt = libusb_get_device_list(jtag_libusb_context, &list);

	for (int idx = 0; idx < cnt; idx++) {
		struct libusb_device *device = list[idx];
		struct libusb_device_descriptor dev_desc;

		int err = libusb_get_device_descriptor(device, &dev_desc);
		if (err != LIBUSB_SUCCESS) {
			LOG_ERROR("libusb_get_device_descriptor() failed with %s", libusb_error_name(err));
			continue;
		}
		LOG_INFO("USB dev VID/PID %x/%x", dev_desc.idVendor, dev_desc.idProduct);
	}
	libusb_free_device_list(list, 1);
}
