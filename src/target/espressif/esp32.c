/***************************************************************************
 *   ESP32 target API for OpenOCD                                          *
 *   Copyright (C) 2016-2019 Espressif Systems Ltd.                        *
 *   Author: Dmitry Yakovlev <dmitry@espressif.com>                        *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include "assert.h"
#include "rtos/rtos.h"
#include "flash/nor/esp_xtensa.h"
#include "esp32.h"
#include "esp32_apptrace.h"
#include "esp_xtensa.h"

/*
This is a JTAG driver for the ESP32, the are two Tensilica cores inside
the ESP32 chip. For more information please have a look into ESP32 target
implementation.
*/

/* ESP32 memory map */
#define ESP32_DRAM_LOW            0x3ffae000
#define ESP32_DRAM_HIGH           0x40000000
#define ESP32_IROM_MASK_LOW       0x40000000
#define ESP32_IROM_MASK_HIGH      0x40064f00
#define ESP32_IRAM_LOW            0x40070000
#define ESP32_IRAM_HIGH           0x400a0000
#define ESP32_RTC_IRAM_LOW        0x400c0000
#define ESP32_RTC_IRAM_HIGH       0x400c2000
#define ESP32_RTC_DRAM_LOW        0x3ff80000
#define ESP32_RTC_DRAM_HIGH       0x3ff82000
#define ESP32_RTC_DATA_LOW        0x50000000
#define ESP32_RTC_DATA_HIGH       0x50002000
#define ESP32_EXTRAM_DATA_LOW     0x3f800000
#define ESP32_EXTRAM_DATA_HIGH    0x3fc00000
#define ESP32_DR_REG_LOW          0x3ff00000
#define ESP32_DR_REG_HIGH         0x3ff71000
#define ESP32_SYS_RAM_LOW         0x60000000UL
#define ESP32_SYS_RAM_HIGH        (ESP32_SYS_RAM_LOW + 0x20000000UL)

/* ESP32 WDT */
#define ESP32_WDT_WKEY_VALUE       0x50d83aa1
#define ESP32_TIMG0_BASE           0x3ff5f000
#define ESP32_TIMG1_BASE           0x3ff60000
#define ESP32_TIMGWDT_CFG0_OFF     0x48
#define ESP32_TIMGWDT_PROTECT_OFF  0x64
#define ESP32_TIMG0WDT_CFG0        (ESP32_TIMG0_BASE + ESP32_TIMGWDT_CFG0_OFF)
#define ESP32_TIMG1WDT_CFG0        (ESP32_TIMG1_BASE + ESP32_TIMGWDT_CFG0_OFF)
#define ESP32_TIMG0WDT_PROTECT     (ESP32_TIMG0_BASE + ESP32_TIMGWDT_PROTECT_OFF)
#define ESP32_TIMG1WDT_PROTECT     (ESP32_TIMG1_BASE + ESP32_TIMGWDT_PROTECT_OFF)
#define ESP32_RTCCNTL_BASE         0x3ff48000
#define ESP32_RTCWDT_CFG_OFF       0x8C
#define ESP32_RTCWDT_PROTECT_OFF   0xA4
#define ESP32_RTCWDT_CFG           (ESP32_RTCCNTL_BASE + ESP32_RTCWDT_CFG_OFF)
#define ESP32_RTCWDT_PROTECT       (ESP32_RTCCNTL_BASE + ESP32_RTCWDT_PROTECT_OFF)

#define ESP32_TRACEMEM_BLOCK_SZ    0x4000

/* ESP32 dport regs */
#define ESP32_DR_REG_DPORT_BASE         ESP32_DR_REG_LOW
#define ESP32_DPORT_APPCPU_CTRL_B_REG   (ESP32_DR_REG_DPORT_BASE + 0x030)
#define ESP32_DPORT_APPCPU_CLKGATE_EN   (1 << 0)
/* ESP32 RTC regs */
#define ESP32_RTC_CNTL_SW_CPU_STALL_REG (ESP32_RTCCNTL_BASE + 0xac)
#define ESP32_RTC_CNTL_SW_CPU_STALL_DEF 0x0


/* Reset ESP32 peripherals.
Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset
How this works:
0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
1. set CPU initial PC to 0x50000000 (ESP32_SMP_RTC_DATA_LOW) by clearing RTC_CNTL_{PRO,APP}CPU_STAT_VECTOR_SEL
2. load stub code into ESP32_SMP_RTC_DATA_LOW; once executed, stub code will disable watchdogs and make CPU spin in an idle loop.
3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
4. wait for the OCD to be reset
5. halt the target and wait for it to be halted (at this point CPU is in the idle loop)
6. restore initial PC and the contents of ESP32_SMP_RTC_DATA_LOW
TODO: some state of RTC_CNTL is not reset during SW_SYS_RST. Need to reset that manually.
*/
static int esp32_soc_reset(struct target *target)
{
	int res;
	struct target_list *head;
	struct xtensa *xtensa;

	LOG_DEBUG("start");
	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("Target not halted before SoC reset, trying to halt it first");
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG("Couldn't halt target before SoC reset, trying to do reset-halt");
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR(
					"Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			int reset_halt_save = target->reset_halt;
			target->reset_halt = 1;
			res = xtensa_deassert_reset(target);
			target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_ERROR(
					"Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			xtensa_halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't halt target before SoC reset");
				return res;
			}
		}
	}
	assert(target->state == TARGET_HALTED);

	if (target->smp) {
		foreach_smp_target(head, target->smp_targets) {
			xtensa = target_to_xtensa(head->target);
			/* if any of the cores is stalled unstall them */
			if (xtensa_dm_core_is_stalled(&xtensa->dbg_mod)) {
				uint32_t word = ESP32_RTC_CNTL_SW_CPU_STALL_DEF;
				LOG_DEBUG("%s: Unstall CPUs before SW reset!",
					target_name(head->target));
				res = xtensa_write_buffer(target,
					ESP32_RTC_CNTL_SW_CPU_STALL_REG,
					sizeof(word),
					(uint8_t *)&word);
				if (res != ERROR_OK) {
					LOG_ERROR("%s: Failed to unstall CPUs before SW reset!",
						target_name(head->target));
					return res;
				}
				break;	/* both cores are unstalled now, so exit the loop */
			}
		}
	}

	/* This this the stub code compiled from esp32_cpu_reset_handler.S.
	   To compile it, run:
	       xtensa-esp32-elf-gcc -c -mtext-section-literals -o stub.o esp32_cpu_reset_handler.S
	       xtensa-esp32-elf-objcopy -j .text -O binary stub.o stub.bin
	   These steps are not included into OpenOCD build process so that a
	   dependency on xtensa-esp32-elf toolchain is not introduced.
	*/
	const uint32_t esp32_reset_stub_code[] = {
		0x00001e06, 0x00001406, 0x3ff48034, 0x3ff480b0,
		0x3ff480b4, 0x3ff48070, 0x00002210, 0x9c492000,
		0x3ff48000, 0x50d83aa1, 0x3ff480a4, 0x3ff5f064,
		0x3ff60064, 0x3ff4808c, 0x3ff5f048, 0x3ff60048,
		0x3ff5a1fc, 0x3ff00038, 0x3ff00030, 0x3ff0002c,
		0x3ff48034, 0x00003000, 0x41305550, 0x0459ffeb,
		0x59ffeb41, 0xffea4104, 0xea410459, 0xffea31ff,
		0xea310439, 0xffea41ff, 0x00000439, 0x6003eb60,
		0x66560461, 0x30555004, 0x41ffe731, 0x0439ffe7,
		0x39ffe741, 0xffe64104, 0xe6410439, 0x410459ff,
		0x0459ffe6, 0x59ffe641, 0xffe54104, 0xe5410459,
		0x410459ff, 0x130cffe5, 0xe4410439, 0x39130cff,
		0x41045904, 0xe331ffe3, 0x006432ff, 0x46007000,
		0x0000fffe,
	};

	LOG_DEBUG("Loading stub code into RTC RAM");
	uint32_t slow_mem_save[sizeof(esp32_reset_stub_code) / sizeof(uint32_t)];

	const int RTC_SLOW_MEM_BASE = 0x50000000;
	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res =
		target_read_buffer(target,
		RTC_SLOW_MEM_BASE,
		sizeof(slow_mem_save),
		(uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to save contents of RTC_SLOW_MEM (%d)!", res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res =
		target_write_buffer(target, RTC_SLOW_MEM_BASE,
		sizeof(esp32_reset_stub_code),
		(const uint8_t *)esp32_reset_stub_code);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write stub (%d)!", res);
		return res;
	}

	LOG_DEBUG("Resuming the target");
	xtensa = target_to_xtensa(target);
	xtensa->suppress_dsr_errors = true;
	res = xtensa_resume(target, 0, RTC_SLOW_MEM_BASE + 4, 0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to run stub (%d)!", res);
		return res;
	}
	xtensa->suppress_dsr_errors = false;
	LOG_DEBUG("resume done, waiting for the target to come alive");

	/* Wait for SoC to reset */
	alive_sleep(100);
	int timeout = 100;
	while (target->state != TARGET_RESET && target->state !=
		TARGET_RUNNING && --timeout > 0) {
		alive_sleep(10);
		xtensa_poll(target);
	}
	if (timeout == 0) {
		LOG_ERROR("Timed out waiting for CPU to be reset, target state=%d", target->state);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	LOG_DEBUG("halting the target");
	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("Timed out waiting for CPU to be halted after SoC reset");
		return res;
	}

	/* Restore the original contents of RTC_SLOW_MEM */
	LOG_DEBUG("restoring RTC_SLOW_MEM");
	res =
		target_write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save),
		(const uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to restore contents of RTC_SLOW_MEM (%d)!", res);
		return res;
	}

	/* Clear memory which is used by RTOS layer to get the task count */
	if (target->rtos && target->rtos->type->post_reset_cleanup) {
		res = (*target->rtos->type->post_reset_cleanup)(target);
		if (res != ERROR_OK)
			LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
	}

	return ERROR_OK;
}

static int esp32_disable_wdts(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32_TIMG0WDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32_TIMG1WDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32_RTCWDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_RTCWDT_CFG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp32_arch_state(struct target *target)
{
	return ERROR_OK;
}

static int esp32_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static int esp32_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	int ret = esp_xtensa_smp_handle_target_event(target, event, priv);
	if (ret != ERROR_OK)
		return ret;

	switch (event) {
	case TARGET_EVENT_HALTED:
		ret = esp32_disable_wdts(target);
		if (ret != ERROR_OK)
			return ret;
		break;
	default:
		break;
	}
	return ERROR_OK;
}

/*
        The TDI pin is also used as a flash Vcc bootstrap pin. If we reset the CPU externally, the last state of the TDI pin can
        allow the power to an 1.8V flash chip to be raised to 3.3V, or the other way around. Users can use the
        esp32 flashbootstrap command to set a level, and this routine will make sure the tdi line will return to
        that when the jtag port is idle.
*/
static void esp32_queue_tdi_idle(struct target *target)
{
	struct esp32_common *esp32 = target_to_esp32(target);
	static uint8_t value;
	uint8_t t[4] = { 0, 0, 0, 0 };

	if (esp32->flash_bootstrap == FBS_TMSLOW) {
		/*Make sure tdi is 0 at the exit of queue execution */
		value = 0;
	} else if (esp32->flash_bootstrap == FBS_TMSHIGH) {
		/*Make sure tdi is 1 at the exit of queue execution */
		value = 1;
	} else {
		return;
	}

	/*Scan out 1 bit, do not move from IRPAUSE after we're done. */
	buf_set_u32(t, 0, 1, value);
	jtag_add_plain_ir_scan(1, t, NULL, TAP_IRPAUSE);
}

static int esp32_target_init(struct command_context *cmd_ctx, struct target *target)
{
	int ret = esp_xtensa_smp_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	ret = target_register_event_callback(esp32_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static const struct xtensa_debug_ops esp32_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static const struct xtensa_power_ops esp32_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static const struct esp_flash_breakpoint_ops esp32_flash_brp_ops = {
	.breakpoint_add = esp_flash_breakpoint_add,
	.breakpoint_remove = esp_flash_breakpoint_remove
};

static const struct esp_xtensa_smp_chip_ops esp32_chip_ops = {
	.reset = esp32_soc_reset
};

static const struct esp_semihost_ops esp32_semihost_ops = {
	.prepare = esp32_disable_wdts
};

static int esp32_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_debug_module_config esp32_dm_cfg = {
		.dbg_ops = &esp32_dbg_ops,
		.pwr_ops = &esp32_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = esp32_queue_tdi_idle,
		.queue_tdi_idle_arg = target
	};

	struct esp32_common *esp32 = calloc(1, sizeof(struct esp32_common));
	if (esp32 == NULL) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_smp_init_arch_info(target, &esp32->esp_xtensa_smp,
		&esp32_dm_cfg, &esp32_flash_brp_ops, &esp32_chip_ops, &esp32_semihost_ops);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32);
		return ret;
	}
	esp32->flash_bootstrap = FBS_DONTCARE;

	/*Assume running target. If different, the first poll will fix this. */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

COMMAND_HELPER(esp32_cmd_flashbootstrap_do, struct esp32_common *esp32)
{
	int state = -1;

	if (CMD_ARGC < 1) {
		const char *st;
		state = esp32->flash_bootstrap;
		if (state == FBS_DONTCARE)
			st = "Don't care";
		else if (state == FBS_TMSLOW)
			st = "Low (3.3V)";
		else if (state == FBS_TMSHIGH)
			st = "High (1.8V)";
		else
			st = "None";
		command_print(CMD, "Current idle tms state: %s", st);
		return ERROR_OK;
	}

	if (!strcasecmp(CMD_ARGV[0], "none"))
		state = FBS_DONTCARE;
	else if (!strcasecmp(CMD_ARGV[0], "1.8"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "3.3"))
		state = FBS_TMSLOW;
	else if (!strcasecmp(CMD_ARGV[0], "high"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "low"))
		state = FBS_TMSLOW;

	if (state == -1) {
		command_print(CMD,
			"Argument unknown. Please pick one of none, high, low, 1.8 or 3.3");
		return ERROR_FAIL;
	}
	esp32->flash_bootstrap = state;
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_flashbootstrap)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp32_cmd_flashbootstrap_do,
				target_to_esp32(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp32_cmd_flashbootstrap_do,
		target_to_esp32(target));
}

static const struct command_registration esp32_any_command_handlers[] = {
	{
		.name = "flashbootstrap",
		.handler = esp32_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help =
			"Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	COMMAND_REGISTRATION_DONE
};

extern const struct command_registration semihosting_common_handlers[];
static const struct command_registration esp32_command_handlers[] = {
	{
		.usage = "",
		.chain = esp_xtensa_smp_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = smp_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = semihosting_common_handlers
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_legacy_command_handlers[] = {
	{
		.name = "esp32",
		.usage = "",
		.chain = esp_xtensa_smp_xtensa_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp_xtensa_smp_esp_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_all_command_handlers[] = {
	{
		.usage = "",
		.chain = esp32_command_handlers,
	},
	{
		.usage = "",
		.chain = esp32_legacy_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for Xtensa targets. */
struct target_type esp32_target = {
	.name = "esp32",

	.poll = esp_xtensa_smp_poll,
	.arch_state = esp32_arch_state,

	.halt = xtensa_halt,
	.resume = esp_xtensa_smp_resume,
	.step = esp_xtensa_smp_step,

	.assert_reset = esp_xtensa_smp_assert_reset,
	.deassert_reset = esp_xtensa_smp_deassert_reset,

	.virt2phys = esp32_virt2phys,
	.mmu = xtensa_mmu_is_enabled,
	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.checksum_memory = xtensa_checksum_memory,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.run_algorithm = xtensa_run_algorithm,
	.start_algorithm = xtensa_start_algorithm,
	.wait_algorithm = xtensa_wait_algorithm,

	.add_breakpoint = esp_xtensa_breakpoint_add,
	.remove_breakpoint = esp_xtensa_breakpoint_remove,

	.add_watchpoint = esp_xtensa_smp_watchpoint_add,
	.remove_watchpoint = esp_xtensa_smp_watchpoint_remove,

	.target_create = esp32_target_create,
	.init_target = esp32_target_init,
	.examine = xtensa_examine,
	.deinit_target = esp_xtensa_target_deinit,

	.gdb_query_custom = xtensa_gdb_query_custom,

	.commands = esp32_all_command_handlers,
};
