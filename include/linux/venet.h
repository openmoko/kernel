/*
 * Copyright 2009 Novell.  All Rights Reserved.
 *
 * Virtual-Ethernet adapter
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

#ifndef _LINUX_VENET_H
#define _LINUX_VENET_H

#include <linux/types.h>

#define VENET_VERSION 1

#define VENET_TYPE "virtual-ethernet"

#define VENET_QUEUE_RX 0
#define VENET_QUEUE_TX 1

struct venet_capabilities {
	__u32 gid;
	__u32 bits;
};

#define VENET_CAP_GROUP_SG 0

/* CAPABILITIES-GROUP SG */
#define VENET_CAP_SG     (1 << 0)
#define VENET_CAP_TSO4   (1 << 1)
#define VENET_CAP_TSO6   (1 << 2)
#define VENET_CAP_ECN    (1 << 3)
#define VENET_CAP_UFO    (1 << 4)

struct venet_iov {
	__u32 len;
	__u64 ptr;
};

#define VENET_SG_FLAG_NEEDS_CSUM (1 << 0)
#define VENET_SG_FLAG_GSO        (1 << 1)
#define VENET_SG_FLAG_ECN        (1 << 2)

struct venet_sg {
	__u64            cookie;
	__u32            flags;
	__u32            len;     /* total length of all iovs */
	struct {
		__u16    start;	  /* csum starting position */
		__u16    offset;  /* offset to place csum */
	} csum;
	struct {
#define VENET_GSO_TYPE_TCPV4	0	/* IPv4 TCP (TSO) */
#define VENET_GSO_TYPE_UDP	1	/* IPv4 UDP (UFO) */
#define VENET_GSO_TYPE_TCPV6	2	/* IPv6 TCP */
		__u8     type;
		__u16    hdrlen;
		__u16    size;
	} gso;
	__u32            count;   /* nr of iovs */
	struct venet_iov iov[1];
};

#define VENET_FUNC_LINKUP   0
#define VENET_FUNC_LINKDOWN 1
#define VENET_FUNC_MACQUERY 2
#define VENET_FUNC_NEGCAP   3 /* negotiate capabilities */
#define VENET_FUNC_FLUSHRX  4

#endif /* _LINUX_VENET_H */
