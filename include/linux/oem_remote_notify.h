/*===========================================================================

         OEM REMOTE NOTIFY SERVICES   Header file


This software is contributed or developed by KYOCERA Corporation.
(C) 2013 KYOCERA Corporation
===========================================================================*/

#ifndef __LINUX_OEM_REMOTE_NOTIFY_H_INCLUDED
#define __LINUX_OEM_REMOTE_NOTIFY_H_INCLUDED

enum incall_status
{
	INCALL_STAT_HOOK = 0,
	INCALL_STAT_TALK = 1,
	INCALL_STAT_INVALID = 2,
};

enum evdo_status
{
	EVDO_STAT_OFF = 0,
	EVDO_STAT_ON = 1,
	EVDO_STAT_INVALID = 2,
};

struct modem_status {
	enum incall_status	stat_1x;
	enum evdo_status	stat_evdo;
};

struct incall_status_data {
	uint8_t	stat_1x;
	uint8_t	stat_evdo;
};


extern struct modem_status oem_remote_notify_get_modem_status(void);


#endif /* __LINUX_OEM_REMOTE_NOTIFY_H_INCLUDED */
