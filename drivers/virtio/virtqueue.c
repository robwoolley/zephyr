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
 */

/*
 * Implements the virtqueue interface as basically described
 * in the original VirtIO paper.
 */

#include <drivers/virtio/virtqueue.h>

#ifdef CONFIG_X86
#include <x86_mmu.h>
#endif

#ifndef CONFIG_VIRTIO_LOG_LEVEL
#define CONFIG_VIRTIO_LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#endif
LOG_MODULE_DECLARE(virtio, CONFIG_VIRTIO_LOG_LEVEL);

struct virtqueue {
	device_t		 vq_dev;
	uint16_t		 vq_queue_index;
	uint16_t		 vq_nentries;
	uint32_t		 vq_flags;
#define	VIRTQUEUE_FLAG_MODERN	 0x0001
#define	VIRTQUEUE_FLAG_INDIRECT	 0x0002
#define	VIRTQUEUE_FLAG_EVENT_IDX 0x0004

	int			 vq_max_indirect_size;
	bus_size_t		 vq_notify_offset;
	virtqueue_intr_t	*vq_intrhand;
	void			*vq_intrhand_arg;

	struct vring		 vq_ring;
	uint16_t		 vq_free_cnt;
	uint16_t		 vq_queued_cnt;
	/*
	 * Head of the free chain in the descriptor table. If
	 * there are no free descriptors, this will be set to
	 * VQ_RING_DESC_CHAIN_END.
	 */
	uint16_t		 vq_desc_head_idx;
	/*
	 * Last consumed descriptor in the used table,
	 * trails vq_ring.used->idx.
	 */
	uint16_t		 vq_used_cons_idx;

	void			*vq_ring_mem;
	int			 vq_indirect_mem_size;
	int			 vq_alignment;
	int			 vq_ring_size;
	char			 vq_name[VIRTQUEUE_MAX_NAME_SZ];

	struct vq_desc_extra {
		void		  *cookie;
		struct vring_desc *indirect;
		vm_paddr_t	   indirect_paddr;
		uint16_t	   ndescs;
	} vq_descx[0];
};

/*
 * The maximum virtqueue size is 2^15. Use that value as the end of
 * descriptor chain terminator since it will never be a valid index
 * in the descriptor table. This is used to verify we are correctly
 * handling vq_free_cnt.
 */
#define VQ_RING_DESC_CHAIN_END 32768

#define VQASSERT(_vq, _exp, _msg, ...)				\
    do {                                                        \
    __ASSERT((_exp),("%s: %s - "_msg, __func__, (_vq)->vq_name,	\
	##__VA_ARGS__))                                         \
    } while (false)

#define VQ_RING_ASSERT_VALID_IDX(_vq, _idx)			\
    VQASSERT((_vq), (_idx) < (_vq)->vq_nentries,		\
	"invalid ring index: %d, max: %d", (_idx),		\
	(_vq)->vq_nentries)

#define VQ_RING_ASSERT_CHAIN_TERM(_vq)				\
    VQASSERT((_vq), (_vq)->vq_desc_head_idx ==			\
	VQ_RING_DESC_CHAIN_END,	"full ring terminated "		\
	"incorrectly: head idx: %d", (_vq)->vq_desc_head_idx)

static int	virtqueue_init_indirect(struct virtqueue *vq, int);
static void	virtqueue_free_indirect(struct virtqueue *vq);
static void	virtqueue_init_indirect_list(struct virtqueue *,
		    struct vring_desc *);

static void	vq_ring_init(struct virtqueue *);
static void	vq_ring_update_avail(struct virtqueue *, uint16_t);
static uint16_t	vq_ring_enqueue_segments(struct virtqueue *,
		    struct vring_desc *, uint16_t, struct sglist *, int, int);
static int	vq_ring_use_indirect(struct virtqueue *, int);
static void	vq_ring_enqueue_indirect(struct virtqueue *, void *,
		    struct sglist *, int, int);
static int	vq_ring_enable_interrupt(struct virtqueue *, uint16_t);
static int	vq_ring_must_notify_host(struct virtqueue *);
static void	vq_ring_notify_host(struct virtqueue *);
static void	vq_ring_free_chain(struct virtqueue *, uint16_t);

#define vq_modern(_vq) 		(((_vq)->vq_flags & VIRTQUEUE_FLAG_MODERN) != 0)
#define vq_htog16(_vq, _val) 	virtio_htog16(vq_modern(_vq), _val)
#define vq_htog32(_vq, _val) 	virtio_htog32(vq_modern(_vq), _val)
#define vq_htog64(_vq, _val) 	virtio_htog64(vq_modern(_vq), _val)
#define vq_gtoh16(_vq, _val) 	virtio_gtoh16(vq_modern(_vq), _val)
#define vq_gtoh32(_vq, _val) 	virtio_gtoh32(vq_modern(_vq), _val)
#define vq_gtoh64(_vq, _val) 	virtio_gtoh64(vq_modern(_vq), _val)

#define Z_VIRTIO_RAM_START DT_REG_ADDR(DT_NODELABEL(virtio_dram))
#define Z_VIRTIO_RAM_SIZE DT_REG_SIZE(DT_NODELABEL(virtio_dram))

/* TOOD: Increase the number of pages in the MMU to be able to map
 *	 a larger portion of memory
 */
#if Z_VIRTIO_RAM_SIZE > 1024 * 1024 * 3
#undef Z_VIRTIO_RAM_SIZE
#define Z_VIRTIO_RAM_SIZE 1024 * 1024 * 3
#endif

struct k_heap VRING_MEM;
static uint8_t *VRING_MEM_BASE;
int
virtqueue_alloc(device_t dev, uint16_t queue, uint16_t size,
    bus_size_t notify_offset, int align, vm_paddr_t highaddr,
    struct vq_alloc_info *info, struct virtqueue **vqp)
{
	struct virtqueue *vq;
	int error;

	*vqp = NULL;
	error = 0;

	virtio_bus_api_t *api = (virtio_bus_api_t *)dev->api;

	/* If the device is x86 based with an MMU map the physical address
	 * otherwise just use the physical address
	 */
#ifdef CONFIG_X86_MMU
	z_phys_map(&VRING_MEM_BASE, Z_VIRTIO_RAM_START, Z_VIRTIO_RAM_SIZE,
			K_MEM_PERM_RW | K_MEM_CACHE_NONE);
#else
	VRING_MEM_BASE = (uint8_t *)Z_VIRTIO_RAM_START;
#endif

	k_heap_init(&VRING_MEM, (void *)VRING_MEM_BASE, Z_VIRTIO_RAM_SIZE);

	if (size == 0) {
		LOG_INF("Virtqueue %d (%s) size is zero", queue,
						info->vqai_name);
		return -ENODEV;
	} else if (!is_power_of_two(size)) {
		LOG_INF("Virtqueue %d (%s) size is not a power of 2: %d",
			queue, info->vqai_name, size);
		return -ENXIO;
	}

	vq = k_heap_alloc(&VRING_MEM, sizeof(struct virtqueue) +
				size * sizeof(struct vq_desc_extra),
				K_FOREVER);

	if (vq == NULL) {
		LOG_ERR("Unable to create virtqueue object");
		error = -ENOMEM;
		goto fail;
	}

	vq->vq_dev = dev;
	strncpy(vq->vq_name, info->vqai_name, sizeof(vq->vq_name));
	vq->vq_queue_index = queue;
	vq->vq_notify_offset = notify_offset;
	vq->vq_alignment = align;
	vq->vq_nentries = size;
	vq->vq_free_cnt = size;
	vq->vq_intrhand = info->vqai_intr;
	vq->vq_intrhand_arg = info->vqai_intr_arg;

	if (api->with_feature(dev, VIRTIO_F_VERSION_1))
		vq->vq_flags |= VIRTQUEUE_FLAG_MODERN;
	if (api->with_feature(dev, VIRTIO_RING_F_EVENT_IDX))
		vq->vq_flags |= VIRTQUEUE_FLAG_EVENT_IDX;

	if (info->vqai_maxindirsz > 1) {
		error = virtqueue_init_indirect(vq, info->vqai_maxindirsz);
		if (error)
			goto fail;
	}

	vq->vq_ring_size = round_page(vring_size(size, align));
	vq->vq_ring_mem = k_heap_aligned_alloc(&VRING_MEM, align,
		vq->vq_ring_size, K_FOREVER);

	if (vq->vq_ring_mem == NULL) {
		LOG_ERR("Virtqueue %d(%s) cannot allocate %d bytes for ring",
			vq->vq_queue_index, vq->vq_name, vq->vq_ring_size);
		error = -ENOMEM;
		goto fail;
	}

	vq_ring_init(vq);
	virtqueue_disable_intr(vq);

	*vqp = vq;

fail:
	if (error)
		virtqueue_free(vq);

	return (error);
}

static int
virtqueue_init_indirect(struct virtqueue *vq, int indirect_size)
{
	device_t dev;
	struct vq_desc_extra *dxp;
	virtio_bus_api_t *api;
	int i, size;

	dev = vq->vq_dev;
	api = (virtio_bus_api_t *)dev->api;

	if (!(api->with_feature(dev, VIRTIO_RING_F_INDIRECT_DESC))) {
		LOG_WRN("Virtqueue %d (%s) requested indirect descriptor but "
			"not negotiated", vq->vq_queue_index, vq->vq_name);
		return (0);
	}

	size = indirect_size * sizeof(struct vring_desc);
	vq->vq_max_indirect_size = indirect_size;
	vq->vq_indirect_mem_size = size;
	vq->vq_flags |= VIRTQUEUE_FLAG_INDIRECT;

	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];

		dxp->indirect = k_malloc(size);

		if (dxp->indirect == NULL) {
			LOG_ERR("Cannot allocate direct list, out of memory");
			return -ENOMEM;
		}

		dxp->indirect_paddr = z_mem_phys_addr(dxp->indirect);
		virtqueue_init_indirect_list(vq, dxp->indirect);
	}

	return (0);
}

static void
virtqueue_free_indirect(struct virtqueue *vq)
{
	struct vq_desc_extra *dxp;
	int i;

	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];

		if (dxp->indirect == NULL)
			break;

		k_free(dxp->indirect);
		dxp->indirect = NULL;
		dxp->indirect_paddr = 0;
	}

	vq->vq_flags &= ~VIRTQUEUE_FLAG_INDIRECT;
	vq->vq_indirect_mem_size = 0;
}

static void
virtqueue_init_indirect_list(struct virtqueue *vq,
    struct vring_desc *indirect)
{
	int i;

	memset(indirect, 0, vq->vq_indirect_mem_size);

	for (i = 0; i < vq->vq_max_indirect_size - 1; i++)
		indirect[i].next = vq_gtoh16(vq, i + 1);
	indirect[i].next = vq_gtoh16(vq, VQ_RING_DESC_CHAIN_END);
}

int
virtqueue_reinit(struct virtqueue *vq, uint16_t size)
{
	struct vq_desc_extra *dxp;
	int i;

	if (vq->vq_nentries != size) {
		LOG_ERR("Cannot reinitialize virtqueue %s with size %d, old"
			" size %d",
			vq->vq_name, size, vq->vq_nentries);
		return -EINVAL;
	}

	/* Warn if the virtqueue was not properly cleaned up. */
	if (vq->vq_free_cnt != vq->vq_nentries) {
		uint16_t leaked_entries = vq->vq_nentries - vq->vq_free_cnt;

		LOG_WRN("Virtqueue %s was not properly emptied. Leaking %d "
			"entries", vq->vq_name, leaked_entries);
	}

	vq->vq_desc_head_idx = 0;
	vq->vq_used_cons_idx = 0;
	vq->vq_queued_cnt = 0;
	vq->vq_free_cnt = vq->vq_nentries;

	/* To be safe, reset all our allocated memory. */
	memset(vq->vq_ring_mem, 0, vq->vq_ring_size);
	for (i = 0; i < vq->vq_nentries; i++) {
		dxp = &vq->vq_descx[i];
		dxp->cookie = NULL;
		dxp->ndescs = 0;
		if (vq->vq_flags & VIRTQUEUE_FLAG_INDIRECT)
			virtqueue_init_indirect_list(vq, dxp->indirect);
	}

	vq_ring_init(vq);
	virtqueue_disable_intr(vq);

	return (0);
}

void
virtqueue_free(struct virtqueue *vq)
{

	if (vq->vq_free_cnt != vq->vq_nentries) {
		LOG_WRN("Virtqueue %d (%s) freeing non-empty virtqueues, leaking"
			"%d entries", vq->vq_queue_index, vq->vq_name,
			vq->vq_nentries - vq->vq_free_cnt);
	}

	if (vq->vq_flags & VIRTQUEUE_FLAG_INDIRECT)
		virtqueue_free_indirect(vq);

	if (vq->vq_ring_mem != NULL) {
		k_heap_free(&VRING_MEM, vq->vq_ring_mem);
		vq->vq_ring_size = 0;
		vq->vq_ring_mem = NULL;
	}

	k_heap_free(&VRING_MEM, vq);
}

vm_paddr_t
virtqueue_paddr(struct virtqueue *vq)
{
	uint32_t offset = (uintptr_t)vq->vq_ring_mem - (uintptr_t)VRING_MEM_BASE;
	return Z_VIRTIO_RAM_START + offset;
}

vm_paddr_t
virtqueue_desc_paddr(struct virtqueue *vq)
{
	uint32_t offset = (uintptr_t)vq->vq_ring.desc - (uintptr_t)VRING_MEM_BASE;
	return Z_VIRTIO_RAM_START + offset;
}

vm_paddr_t
virtqueue_avail_paddr(struct virtqueue *vq)
{
	uint32_t offset = (uintptr_t)vq->vq_ring.avail - (uintptr_t)VRING_MEM_BASE;
	return Z_VIRTIO_RAM_START + offset;
}

vm_paddr_t
virtqueue_used_paddr(struct virtqueue *vq)
{
	uint32_t offset = (uintptr_t)vq->vq_ring.used - (uintptr_t)VRING_MEM_BASE;
	return Z_VIRTIO_RAM_START + offset;
}

uint16_t
virtqueue_index(struct virtqueue *vq)
{

	return (vq->vq_queue_index);
}

int
virtqueue_size(struct virtqueue *vq)
{

	return (vq->vq_nentries);
}

int
virtqueue_nfree(struct virtqueue *vq)
{

	return (vq->vq_free_cnt);
}

int
virtqueue_empty(struct virtqueue *vq)
{

	return (vq->vq_nentries == vq->vq_free_cnt);
}

int
virtqueue_full(struct virtqueue *vq)
{

	return (vq->vq_free_cnt == 0);
}

void
virtqueue_notify(struct virtqueue *vq)
{

	/* Ensure updated avail->idx is visible to host. */
	VIRTIO_MEMORY_BARRIER();

	if (vq_ring_must_notify_host(vq))
		vq_ring_notify_host(vq);
	vq->vq_queued_cnt = 0;
}

int
virtqueue_nused(struct virtqueue *vq)
{
	uint16_t used_idx, nused;

	used_idx = vq_htog16(vq, vq->vq_ring.used->idx);

	nused = (uint16_t)(used_idx - vq->vq_used_cons_idx);
	VQASSERT(vq, nused <= vq->vq_nentries, "used more than available");

	return (nused);
}

int
virtqueue_intr_filter(struct virtqueue *vq)
{

	if (vq->vq_used_cons_idx == vq_htog16(vq, vq->vq_ring.used->idx))
		return (0);

	virtqueue_disable_intr(vq);

	return (1);
}

void
virtqueue_intr(struct virtqueue *vq)
{
	while (1) {
		if (vq->vq_intrhand != NULL) {
			vq->vq_intrhand(vq->vq_intrhand_arg);
		} else {
			LOG_WRN("No interrupt handler for queue %d",
					vq->vq_queue_index);
			return;
		}

		/* if there is still work return to the interrupt handle */
		if (!virtqueue_enable_intr(vq)) {
			break;
		}
	}
}

int
virtqueue_enable_intr(struct virtqueue *vq)
{

	return (vq_ring_enable_interrupt(vq, 0));
}

int
virtqueue_postpone_intr(struct virtqueue *vq, vq_postpone_t hint)
{
	uint16_t ndesc, avail_idx;

	avail_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
	ndesc = (uint16_t)(avail_idx - vq->vq_used_cons_idx);

	switch (hint) {
	case VQ_POSTPONE_SHORT:
		ndesc = ndesc / 4;
		break;
	case VQ_POSTPONE_LONG:
		ndesc = (ndesc * 3) / 4;
		break;
	case VQ_POSTPONE_EMPTIED:
		break;
	}

	return (vq_ring_enable_interrupt(vq, ndesc));
}

/*
 * Note this is only considered a hint to the host.
 */
void
virtqueue_disable_intr(struct virtqueue *vq)
{

	if (vq->vq_flags & VIRTQUEUE_FLAG_EVENT_IDX) {
		vring_used_event(&vq->vq_ring) = vq_gtoh16(vq,
		    vq->vq_used_cons_idx - vq->vq_nentries - 1);
		return;
	}

	vq->vq_ring.avail->flags |= vq_gtoh16(vq, VRING_AVAIL_F_NO_INTERRUPT);
}

int
virtqueue_enqueue(struct virtqueue *vq, void *cookie, struct sglist *sg,
    int readable, int writable)
{
	struct vq_desc_extra *dxp;
	int needed;
	uint16_t head_idx, idx;

	needed = readable + writable;

	LOG_DBG("%s(%d): readable: %d, writable: %d, needed: %d\n", __func__, __LINE__, readable, writable, needed);

	VQASSERT(vq, cookie != NULL, "enqueuing with no cookie");
	VQASSERT(vq, needed == sg->sg_nseg,
	    "segment count mismatch, %d, %d", needed, sg->sg_nseg);
	VQASSERT(vq,
	    needed <= vq->vq_nentries || needed <= vq->vq_max_indirect_size,
	    "too many segments to enqueue: %d, %d/%d", needed,
	    vq->vq_nentries, vq->vq_max_indirect_size);

	if (needed < 1)
		return -EINVAL;
	if (vq->vq_free_cnt == 0)
		return -ENOSPC;

	if (vq_ring_use_indirect(vq, needed)) {
		vq_ring_enqueue_indirect(vq, cookie, sg, readable, writable);
		return (0);
	} else if (vq->vq_free_cnt < needed)
		return -EMSGSIZE;

	head_idx = vq->vq_desc_head_idx;
	VQ_RING_ASSERT_VALID_IDX(vq, head_idx);
	dxp = &vq->vq_descx[head_idx];

	VQASSERT(vq, dxp->cookie == NULL,
	    "cookie already exists for index %d", head_idx);
	dxp->cookie = cookie;
	dxp->ndescs = needed;

	idx = vq_ring_enqueue_segments(vq, vq->vq_ring.desc, head_idx,
	    sg, readable, writable);

	vq->vq_desc_head_idx = idx;
	vq->vq_free_cnt -= needed;
	if (vq->vq_free_cnt == 0)
		VQ_RING_ASSERT_CHAIN_TERM(vq);
	else
		VQ_RING_ASSERT_VALID_IDX(vq, idx);

	vq_ring_update_avail(vq, head_idx);

	return (0);
}

void *
virtqueue_dequeue(struct virtqueue *vq, uint32_t *len)
{
	struct vring_used_elem *uep;
	void *cookie;
	uint16_t used_idx, desc_idx;

	if (vq->vq_used_cons_idx == vq_htog16(vq, vq->vq_ring.used->idx))
		return (NULL);

	used_idx = vq->vq_used_cons_idx++ & (vq->vq_nentries - 1);
	uep = &vq->vq_ring.used->ring[used_idx];

	VIRTIO_MEMORY_BARRIER();
	desc_idx = (uint16_t) vq_htog32(vq, uep->id);
	if (len != NULL)
		*len = vq_htog32(vq, uep->len);

	vq_ring_free_chain(vq, desc_idx);

	cookie = vq->vq_descx[desc_idx].cookie;
	VQASSERT(vq, cookie != NULL, "no cookie for index %d", desc_idx);
	vq->vq_descx[desc_idx].cookie = NULL;

	return (cookie);
}

void *
virtqueue_poll(struct virtqueue *vq, uint32_t *len)
{
	void *cookie;

	while ((cookie = virtqueue_dequeue(vq, len)) == NULL) {
		k_busy_wait(100);
	}
#if 0
	VIRTIO_BUS_POLL(vq->vq_dev);
	while ((cookie = virtqueue_dequeue(vq, len)) == NULL) {
		cpu_spinwait();
		VIRTIO_BUS_POLL(vq->vq_dev);
#endif /* 0 */

	return (cookie);
}

void *
virtqueue_drain(struct virtqueue *vq, int *last)
{
	void *cookie;
	int idx;

	cookie = NULL;
	idx = *last;

	while (idx < vq->vq_nentries && cookie == NULL) {
		if ((cookie = vq->vq_descx[idx].cookie) != NULL) {
			vq->vq_descx[idx].cookie = NULL;
			/* Free chain to keep free count consistent. */
			vq_ring_free_chain(vq, idx);
		}
		idx++;
	}

	*last = idx;

	return (cookie);
}

void
virtqueue_dump(struct virtqueue *vq)
{

	if (vq == NULL)
		return;

	printk("VQ: %s - size=%d; free=%d; used=%d; queued=%d; "
	    "desc_head_idx=%d; avail.idx=%d; used_cons_idx=%d; "
	    "used.idx=%d; used_event_idx=%d; avail.flags=0x%x; used.flags=0x%x\n",
	    vq->vq_name, vq->vq_nentries, vq->vq_free_cnt, virtqueue_nused(vq),
	    vq->vq_queued_cnt, vq->vq_desc_head_idx,
	    vq_htog16(vq, vq->vq_ring.avail->idx), vq->vq_used_cons_idx,
	    vq_htog16(vq, vq->vq_ring.used->idx),
	    vq_htog16(vq, vring_used_event(&vq->vq_ring)),
	    vq_htog16(vq, vq->vq_ring.avail->flags),
	    vq_htog16(vq, vq->vq_ring.used->flags));
}

static void
vq_ring_init(struct virtqueue *vq)
{
	struct vring *vr;
	char *ring_mem;
	int i, size;

	ring_mem = vq->vq_ring_mem;
	size = vq->vq_nentries;
	vr = &vq->vq_ring;

	vring_init(vr, size, ring_mem, vq->vq_alignment);

	for (i = 0; i < size - 1; i++)
		vr->desc[i].next = vq_gtoh16(vq, i + 1);
	vr->desc[i].next = vq_gtoh16(vq, VQ_RING_DESC_CHAIN_END);
}

static void
vq_ring_update_avail(struct virtqueue *vq, uint16_t desc_idx)
{
	uint16_t avail_idx, avail_ring_idx;

	/*
	 * Place the head of the descriptor chain into the next slot and make
	 * it usable to the host. The chain is made available now rather than
	 * deferring to virtqueue_notify() in the hopes that if the host is
	 * currently running on another CPU, we can keep it processing the new
	 * descriptor.
	 */
	avail_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
	avail_ring_idx = avail_idx & (vq->vq_nentries - 1);
	vq->vq_ring.avail->ring[avail_ring_idx] = vq_gtoh16(vq, desc_idx);

	/* TODO: Look to implement a hardware memory barrier if possible.
	 *	Zephyr does not currently have framework support for barriers.
	 */
	VIRTIO_MEMORY_BARRIER();

	vq->vq_ring.avail->idx = vq_gtoh16(vq, avail_idx + 1);

	/* Keep pending count until virtqueue_notify(). */
	vq->vq_queued_cnt++;
}

static uint16_t
vq_ring_enqueue_segments(struct virtqueue *vq, struct vring_desc *desc,
    uint16_t head_idx, struct sglist *sg, int readable, int writable)
{
	struct sglist_seg *seg;
	struct vring_desc *dp;
	int i, needed;
	uint16_t idx;

	needed = readable + writable;

	for (i = 0, idx = head_idx, seg = sg->sg_segs;
	     i < needed;
	     i++, idx = vq_htog16(vq, dp->next), seg++) {
		LOG_DBG("%s(%d): i:%d seg:%p\n", __func__, __LINE__, i, seg);
		LOG_DBG("%s(%d): seg ss_paddr:%lx ss_len:%lu",
			__func__, __LINE__,
			seg->ss_paddr, seg->ss_len);
		VQASSERT(vq, idx != VQ_RING_DESC_CHAIN_END,
		    "premature end of free desc chain");

		dp = &desc[idx];
		dp->addr = vq_gtoh64(vq, seg->ss_paddr);
		dp->len = vq_gtoh32(vq, seg->ss_len);
		dp->flags = 0;
		LOG_DBG("%s(%d): dp addr:%llx len:%d flags:%d\n",
			__func__, __LINE__,
			dp->addr, dp->len, dp->flags);

		if (i < needed - 1)
			dp->flags |= vq_gtoh16(vq, VRING_DESC_F_NEXT);
		if (i >= readable)
			dp->flags |= vq_gtoh16(vq, VRING_DESC_F_WRITE);
	}

	return (idx);
}

static int
vq_ring_use_indirect(struct virtqueue *vq, int needed)
{

	if ((vq->vq_flags & VIRTQUEUE_FLAG_INDIRECT) == 0)
		return (0);

	if (vq->vq_max_indirect_size < needed)
		return (0);

	if (needed < 2)
		return (0);

	return (1);
}

static void
vq_ring_enqueue_indirect(struct virtqueue *vq, void *cookie,
    struct sglist *sg, int readable, int writable)
{
	struct vring_desc *dp;
	struct vq_desc_extra *dxp;
	int needed;
	uint16_t head_idx;

	needed = readable + writable;
	VQASSERT(vq, needed <= vq->vq_max_indirect_size,
	    "enqueuing too many indirect descriptors");

	head_idx = vq->vq_desc_head_idx;
	VQ_RING_ASSERT_VALID_IDX(vq, head_idx);
	dp = &vq->vq_ring.desc[head_idx];
	dxp = &vq->vq_descx[head_idx];

	VQASSERT(vq, dxp->cookie == NULL,
	    "cookie already exists for index %d", head_idx);
	dxp->cookie = cookie;
	dxp->ndescs = 1;

/* No longer needed?
	uintptr_t paddr;
#ifdef CONFIG_X86_MMU
	paddr = z_mem_phys_addr(dxp->indirect);
#else
	paddr = (uintptr_t)dxp->indirect;
#endif
	dp->addr = vq_gtoh64(vq, paddr);
*/
	dp->addr = vq_gtoh64(vq, dxp->indirect_paddr);
	dp->len = vq_gtoh32(vq, needed * sizeof(struct vring_desc));
	dp->flags = vq_gtoh16(vq, VRING_DESC_F_INDIRECT);

	vq_ring_enqueue_segments(vq, dxp->indirect, 0,
	    sg, readable, writable);

	vq->vq_desc_head_idx = vq_htog16(vq, dp->next);
	vq->vq_free_cnt--;
	if (vq->vq_free_cnt == 0)
		VQ_RING_ASSERT_CHAIN_TERM(vq);
	else
		VQ_RING_ASSERT_VALID_IDX(vq, vq->vq_desc_head_idx);

	vq_ring_update_avail(vq, head_idx);
}

static int
vq_ring_enable_interrupt(struct virtqueue *vq, uint16_t ndesc)
{

	/*
	 * Enable interrupts, making sure we get the latest index of
	 * what's already been consumed.
	 */
	if (vq->vq_flags & VIRTQUEUE_FLAG_EVENT_IDX) {
		vring_used_event(&vq->vq_ring) =
		    vq_gtoh16(vq, vq->vq_used_cons_idx + ndesc);
	} else {
		vq->vq_ring.avail->flags &=
		    vq_gtoh16(vq, ~VRING_AVAIL_F_NO_INTERRUPT);
	}

	VIRTIO_MEMORY_BARRIER();

	/*
	 * Enough items may have already been consumed to meet our threshold
	 * since we last checked. Let our caller know so it processes the new
	 * entries.
	 */
	if (virtqueue_nused(vq) > ndesc)
		return (1);

	return (0);
}

static int
vq_ring_must_notify_host(struct virtqueue *vq)
{
	uint16_t new_idx, prev_idx, event_idx, flags;

	if (vq->vq_flags & VIRTQUEUE_FLAG_EVENT_IDX) {
		new_idx = vq_htog16(vq, vq->vq_ring.avail->idx);
		prev_idx = new_idx - vq->vq_queued_cnt;

/* the compiler will complain about strict aliasing for this macro
 * ignore that for now
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
		event_idx = vq_htog16(vq, vring_avail_event(&vq->vq_ring));
#pragma GCC diagnostic pop

		return (vring_need_event(event_idx, new_idx, prev_idx) != 0);
	}

	flags = vq->vq_ring.used->flags;
	return ((flags & vq_gtoh16(vq, VRING_USED_F_NO_NOTIFY)) == 0);
}

static void
vq_ring_notify_host(struct virtqueue *vq)
{
	virtio_bus_api_t *api = (virtio_bus_api_t *)vq->vq_dev->api;

	api->notify_virtqueue(
		vq->vq_dev, vq->vq_queue_index, vq->vq_notify_offset
	);
}

static void
vq_ring_free_chain(struct virtqueue *vq, uint16_t desc_idx)
{
	struct vring_desc *dp;
	struct vq_desc_extra *dxp;

	VQ_RING_ASSERT_VALID_IDX(vq, desc_idx);
	dp = &vq->vq_ring.desc[desc_idx];
	dxp = &vq->vq_descx[desc_idx];

	if (vq->vq_free_cnt == 0)
		VQ_RING_ASSERT_CHAIN_TERM(vq);

	vq->vq_free_cnt += dxp->ndescs;
	dxp->ndescs--;

/*
	uint16_t host_supports_indirect = vq_gtoh16(vq, VRING_DESC_F_INDIRECT);
	uint16_t vring_has_next = vq_gtoh16(vq, VRING_DESC_F_NEXT);
	if ((dp->flags & host_supports_indirect) == 0) {
		while (dp->flags & vring_has_next) {
*/

	if ((dp->flags & vq_gtoh16(vq, VRING_DESC_F_INDIRECT)) == 0) {
		while (dp->flags & vq_gtoh16(vq, VRING_DESC_F_NEXT)) {
			uint16_t next_idx = vq_htog16(vq, dp->next);
			VQ_RING_ASSERT_VALID_IDX(vq, next_idx);
			dp = &vq->vq_ring.desc[next_idx];
			dxp->ndescs--;
		}
	}

	VQASSERT(vq, dxp->ndescs == 0,
	    "failed to free entire desc chain, remaining: %d", dxp->ndescs);

	/*
	 * We must append the existing free chain, if any, to the end of
	 * newly freed chain. If the virtqueue was completely used, then
	 * head would be VQ_RING_DESC_CHAIN_END (ASSERTed above).
	 */
	dp->next = vq_gtoh16(vq, vq->vq_desc_head_idx);
	vq->vq_desc_head_idx = desc_idx;
}