/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2011, Bryan Venteicher <bryanv@FreeBSD.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _VIRTIO_VIRTQUEUE_H
#define _VIRTIO_VIRTQUEUE_H
#pragma once

#include <zephyr.h>
#include <drivers/virtio/virtio_ring.h>
#include <drivers/virtio/virtio.h>

struct virtqueue;

/* Device callback for a virtqueue interrupt. */
typedef void virtqueue_intr_t(void *);
struct __virtio_device__;
typedef struct __virtio_device__ virtio_device_t;

#define VIRTQUEUE_MODERN 0x1
#define VIRTQUEUE_INDIRECT 0x2
#define VIRTQUEUE_EVENT_IDX 0x4
/*
 * Hint on how long the next interrupt should be postponed. This is
 * only used when the EVENT_IDX feature is negotiated.
 */
typedef enum {
	VQ_POSTPONE_SHORT,
	VQ_POSTPONE_LONG,
	VQ_POSTPONE_EMPTIED	/* Until all available desc are used. */
} vq_postpone_t;

#define VIRTQUEUE_MAX_NAME_SZ	32

/* One for each virtqueue the device wishes to allocate. */
struct vq_alloc_info {
	char		   vqai_name[VIRTQUEUE_MAX_NAME_SZ];
	int		   vqai_maxindirsz;
	virtqueue_intr_t  *vqai_intr;
	void		  *vqai_intr_arg;
	struct virtqueue **vqai_vq;
};

#define VQ_ALLOC_INFO_INIT(_i,_nsegs,_intr,_arg,_vqp,_str,...) do {	\
	strncpy((_i)->vqai_name, _str, VIRTQUEUE_MAX_NAME_SZ);		\
	(_i)->vqai_maxindirsz = (_nsegs);				\
	(_i)->vqai_intr = (_intr);					\
	(_i)->vqai_intr_arg = (_arg);					\
	(_i)->vqai_vq = (_vqp);						\
} while (0)



typedef struct vq_desc_extra vq_desc_extra_t;


#define virtqueue_modern(virtqueue) (((virtqueue)->flags & VIRTQUEUE_MODERN)\
									 != 0)

int	 virtqueue_alloc(virtio_device_t dev, uint16_t queue, uint16_t size,
	     unsigned long notify_offset, int align, uintptr_t highaddr,
	     struct vq_alloc_info *info, struct virtqueue **vqp);
void	*virtqueue_drain(struct virtqueue *vq, int *last);
void	 virtqueue_free(struct virtqueue *vq);
int	 virtqueue_reinit(struct virtqueue *vq, uint16_t size);

int	 virtqueue_intr_filter(struct virtqueue *vq);
void	 virtqueue_intr(struct virtqueue *vq);
int	 virtqueue_enable_intr(struct virtqueue *vq);
int	 virtqueue_postpone_intr(struct virtqueue *vq, vq_postpone_t hint);
void	 virtqueue_disable_intr(struct virtqueue *vq);

/* Get physical address of the virtqueue ring. */
uintptr_t virtqueue_paddr(struct virtqueue *vq);
uintptr_t virtqueue_desc_paddr(struct virtqueue *vq);
uintptr_t virtqueue_avail_paddr(struct virtqueue *vq);
uintptr_t virtqueue_used_paddr(struct virtqueue *vq);

uint16_t virtqueue_index(struct virtqueue *vq);
int	 virtqueue_full(struct virtqueue *vq);
int	 virtqueue_empty(struct virtqueue *vq);
int	 virtqueue_size(struct virtqueue *vq);
int	 virtqueue_nfree(struct virtqueue *vq);
int	 virtqueue_nused(struct virtqueue *vq);
void	 virtqueue_notify(struct virtqueue *vq);
void	 virtqueue_dump(struct virtqueue *vq);

int	 virtqueue_enqueue(struct virtqueue *vq, void *cookie,
	     sys_slist_t *readable, int readable_len,
	     sys_slist_t *writable, int writable_len);
void	*virtqueue_dequeue(struct virtqueue *vq, uint32_t *len);
void	*virtqueue_poll(struct virtqueue *vq, uint32_t *len);

#endif /* _VIRTIO_VIRTQUEUE_H */
