/*
 * Copyright 2009 Novell.  All Rights Reserved.
 *
 * Author:
 *      Gregory Haskins <ghaskins@novell.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef _LINUX_SHM_SIGNAL_H
#define _LINUX_SHM_SIGNAL_H

#include <linux/types.h>

/*
 *---------
 * The following structures represent data that is shared across boundaries
 * which may be quite disparate from one another (e.g. Windows vs Linux,
 * 32 vs 64 bit, etc).  Therefore, care has been taken to make sure they
 * present data in a manner that is independent of the environment.
 *-----------
 */

#define SHM_SIGNAL_MAGIC 0x58fa39df
#define SHM_SIGNAL_VER   1

struct shm_signal_irq {
	__u8                  enabled;
	__u8                  pending;
	__u8                  dirty;
};

enum shm_signal_locality {
	shm_locality_north,
	shm_locality_south,
};

struct shm_signal_desc {
	__u32                 magic;
	__u32                 ver;
	struct shm_signal_irq irq[2];
};

/* --- END SHARED STRUCTURES --- */

#ifdef __KERNEL__

#include <linux/kref.h>
#include <linux/interrupt.h>

struct shm_signal_notifier {
	void (*signal)(struct shm_signal_notifier *);
};

struct shm_signal;

struct shm_signal_ops {
	int      (*inject)(struct shm_signal *s);
	void     (*fault)(struct shm_signal *s, const char *fmt, ...);
	void     (*release)(struct shm_signal *s);
};

enum {
	shm_signal_in_wakeup,
};

struct shm_signal {
	struct kref                 kref;
	spinlock_t                  lock;
	enum shm_signal_locality    locale;
	unsigned long               flags;
	struct shm_signal_ops      *ops;
	struct shm_signal_desc     *desc;
	struct shm_signal_notifier *notifier;
	struct tasklet_struct       deferred_notify;
};

#define SHM_SIGNAL_FAULT(s, fmt, args...)  \
  ((s)->ops->fault ? (s)->ops->fault((s), fmt, ## args) : panic(fmt, ## args))

 /*
  * These functions should only be used internally
  */
void _shm_signal_release(struct kref *kref);
void _shm_signal_wakeup(struct shm_signal *s);

/**
 * shm_signal_init() - initialize an SHM_SIGNAL
 * @s:        SHM_SIGNAL context
 *
 * Initializes SHM_SIGNAL context before first use
 *
 **/
void shm_signal_init(struct shm_signal *s, enum shm_signal_locality locale,
		     struct shm_signal_ops *ops, struct shm_signal_desc *desc);

/**
 * shm_signal_get() - acquire an SHM_SIGNAL context reference
 * @s:        SHM_SIGNAL context
 *
 **/
static inline struct shm_signal *shm_signal_get(struct shm_signal *s)
{
	kref_get(&s->kref);

	return s;
}

/**
 * shm_signal_put() - release an SHM_SIGNAL context reference
 * @s:        SHM_SIGNAL context
 *
 **/
static inline void shm_signal_put(struct shm_signal *s)
{
	kref_put(&s->kref, _shm_signal_release);
}

/**
 * shm_signal_enable() - enables local notifications on an SHM_SIGNAL
 * @s:        SHM_SIGNAL context
 * @flags:      Reserved for future use, must be 0
 *
 * Enables/unmasks the registered notifier (if applicable) to receive wakeups
 * whenever the remote side performs an shm_signal() operation. A notification
 * will be dispatched immediately if any pending signals have already been
 * issued prior to invoking this call.
 *
 * This is synonymous with unmasking an interrupt.
 *
 * Returns: success = 0, <0 = ERRNO
 *
 **/
int shm_signal_enable(struct shm_signal *s, int flags);

/**
 * shm_signal_disable() - disable local notifications on an SHM_SIGNAL
 * @s:        SHM_SIGNAL context
 * @flags:      Reserved for future use, must be 0
 *
 * Disables/masks the registered shm_signal_notifier (if applicable) from
 * receiving any further notifications.  Any subsequent calls to shm_signal()
 * by the remote side will update the shm as dirty, but will not traverse the
 * locale boundary and will not invoke the notifier callback.  Signals
 * delivered while masked will be deferred until shm_signal_enable() is
 * invoked.
 *
 * This is synonymous with masking an interrupt
 *
 * Returns: success = 0, <0 = ERRNO
 *
 **/
int shm_signal_disable(struct shm_signal *s, int flags);

/**
 * shm_signal_inject() - notify the remote side about shm changes
 * @s:        SHM_SIGNAL context
 * @flags:      Reserved for future use, must be 0
 *
 * Marks the shm state as "dirty" and, if enabled, will traverse
 * a locale boundary to inject a remote notification.  The remote
 * side controls whether the notification should be delivered via
 * the shm_signal_enable/disable() interface.
 *
 * The specifics of how to traverse a locale boundary are abstracted
 * by the shm_signal_ops->signal() interface and provided by a particular
 * implementation.  However, typically going north to south would be
 * something like a syscall/hypercall, and going south to north would be
 * something like a posix-signal/guest-interrupt.
 *
 * Returns: success = 0, <0 = ERRNO
 *
 **/
int shm_signal_inject(struct shm_signal *s, int flags);

#endif /* __KERNEL__ */

#endif /* _LINUX_SHM_SIGNAL_H */
