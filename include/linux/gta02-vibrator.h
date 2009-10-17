#ifndef __LINUX_GTA02_VIBRATOR
#define __LINUX_GTA02_VIBRATOR

struct gta02_vib_platform_data {
	int (* enable_fiq)(void);
	void (*disable_fiq)(void);
	void (*kick_fiq)(void);
};

#endif
