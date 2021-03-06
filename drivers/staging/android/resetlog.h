/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <resetlog.h>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2013 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#ifndef _RESET_LOG_H
#define _RESET_LOG_H

#define HEADER_VERION           "v1.0.4"
#define HEADER_VERION_SIZE      (8)

#define MSM_KCJLOG_BASE         (MSM_UNINIT_RAM_BASE)
#define SIZE_RAM_CONSOLE        (0x00080000)

#define SIZE_SMEM_ALLOC         (512)

#define SIZE_KERNEL_LOG         (SIZE_RAM_CONSOLE - sizeof(ram_console_header_type))
#define SIZE_CONTROL_INFO       (SIZE_SMEM_ALLOC)
#define SIZE_LOGCAT_MAIN        (512 * 1024)
#define SIZE_LOGCAT_SYSTEM      (512 * 1024)
#define SIZE_LOGCAT_EVENTS      (256 * 1024)
#define SIZE_LOGCAT_RADIO       (512 * 1024)

#define ADDR_KERNEL_LOG         (unsigned long)(&((ram_console_type *)MSM_KCJLOG_BASE)->msg[0])
#define ADDR_CONTROL_INFO       (MSM_KCJLOG_BASE    + SIZE_RAM_CONSOLE  )
#define ADDR_LOGCAT_MAIN        (ADDR_CONTROL_INFO  + SIZE_CONTROL_INFO )
#define ADDR_LOGCAT_SYSTEM      (ADDR_LOGCAT_MAIN   + SIZE_LOGCAT_MAIN  )
#define ADDR_LOGCAT_EVENTS      (ADDR_LOGCAT_SYSTEM + SIZE_LOGCAT_SYSTEM)
#define ADDR_LOGCAT_RADIO       (ADDR_LOGCAT_EVENTS + SIZE_LOGCAT_EVENTS)

enum {
	LOGGER_INFO_MAIN,
	LOGGER_INFO_SYSTEM,
	LOGGER_INFO_EVENTS,
	LOGGER_INFO_RADIO,
	LOGGER_INFO_MAX,
};

#define CRASH_MAGIC_CODE         "KC ERROR"

#define CRASH_SYSTEM_KERNEL      "KERNEL"
#define CRASH_SYSTEM_MODEM       "MODEM"
#define CRASH_SYSTEM_RIVA        "RIVA"
#define CRASH_SYSTEM_LPASS       "LPASS"
#define CRASH_SYSTEM_ANDROID     "ANDROID"
#define CRASH_SYSTEM_UNKNOWN     "UNKNOWN"

#define CRASH_KIND_PANIC         "KERNEL PANIC"
#define CRASH_KIND_FATAL         "ERR FATAL"
#define CRASH_KIND_EXEPTION      "EXEPTION"
#define CRASH_KIND_WDOG_HW       "HW WATCH DOG"
#define CRASH_KIND_WDOG_SW       "SW WATCH DOG"
#define CRASH_KIND_SYS_SERVER    "SYSTEM SERVER CRASH"
#define CRASH_KIND_UNKNOWN       "UNKNOWN"

#define RESTART_MODE_USER        (0x52455355)	// "USER"
#define RAM_CONSOLE_MAGIC        (0x43474244)	// "DBGC"

/* RAM_CONSOLE Contol */
typedef struct {
    unsigned char               magic[4];       // RAM_CONSOLE MAGIC("DEBG")
    unsigned long               start;          // RAM_CONSOLE start
    unsigned long               size;           // RAM_CONSOLE size
} ram_console_header_type;

typedef struct {
    ram_console_header_type     header;         // RAM_CONSOLE header
    unsigned char               msg[1];         // RAM_CONSOLE message
} ram_console_type;

/* Log Control */
struct logger_log_info {
	unsigned long           w_off;
	unsigned long           head;
};

#define CRASH_CODE_SIZE         (24)
#define CRASH_TIME_SIZE         (64)
#define VERSION_SIZE            (64)
#define MODEL_SIZE              (32)

typedef struct {
	unsigned char           magic_code[CRASH_CODE_SIZE];
	unsigned char           crash_system[CRASH_CODE_SIZE];
	unsigned char           crash_kind[CRASH_CODE_SIZE];
	unsigned char           crash_time[CRASH_TIME_SIZE];
	unsigned char           linux_ver[VERSION_SIZE];
	unsigned char           modem_ver[VERSION_SIZE];
	unsigned char           model[MODEL_SIZE];
	struct logger_log_info  info[LOGGER_INFO_MAX];
	unsigned long           pErr_F3_Trace_Buffer;
	unsigned long           Sizeof_Err_F3_Trace_Buffer;
	unsigned long           Err_F3_Trace_Buffer_Head;
	unsigned long           Err_F3_Trace_Wrap_Flag;
	unsigned long           pErr_Data;
	unsigned long           Sizeof_Err_Data;
	unsigned long           pSmem_log_events;
	unsigned long           pSmem_log_write_idx;
	unsigned long           Sizeof_Smem_log;
	unsigned long           Sizeof_system_imem_a;
	unsigned long           Sizeof_system_imem_c;
	unsigned long           Sizeof_mm_imem;
	unsigned long           Sizeof_rpm_code_ram;
	unsigned long           Sizeof_rpm_msg_ram;
	unsigned long           Sizeof_lpass_mem;
	unsigned long           Sizeof_cpu_register;
	unsigned long           Sizeof_rpm_wdt_reset;
	unsigned long           Sizeof_app_wdt_reset;
	unsigned long           Sizeof_reset_status;
	unsigned long           restart_mode;
	unsigned long           console_magic;
} ram_log_info_type;


#endif /* _RESET_LOG_H */
