/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2017, Bryan Venteicher <bryanv@FreeBSD.org>
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

/* Copyright (c) 2021, Wind River Systems, Inc.*/

#include <zephyr.h>

#include <drivers/virtio/virtio.h>
#include <drivers/virtio/virtio_config.h>
#include <drivers/virtio/pcie/utils.h>

#include <sys/device_mmio.h>
#include <sys/arch_interface.h>

#ifndef CONFIG_VIRTIO_LOG_LEVEL
#define CONFIG_VIRTIO_LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#endif
LOG_MODULE_DECLARE(virtio, CONFIG_VIRTIO_LOG_LEVEL);

#define VIRTIO_CAP_OFFSET_HEADER_0 0x34
#define VIRTIO_CAP_OFFSET_HEADER_2 0x14

/* Zephyr Values used to enumerate PCI lanes */
#define MAX_BUS (0xFFFFFFFF & PCIE_BDF_BUS_MASK)
#define MAX_DEV (0xFFFFFFFF & PCIE_BDF_DEV_MASK)
#define MAX_FUNC (0xFFFFFFFF & PCIE_BDF_FUNC_MASK)

/* General functions */

bool virtio_pci_with_feature(virtio_device_t *dev, uint64_t feature)
{
	return (dev->data.virtio_features & feature) != 0;
}

static uint32_t virtio_pci_read(const virtio_device_t *dev, uintptr_t offset,
					size_t size)
{
	const virtio_config_t *config = &dev->data;

	uint32_t data;

	if (config->virtio_io_res_type == VIRTIO_IO) {
		switch (size) {
		case 1:
			data = sys_in8(config->virtio_io_res+offset);

			break;
		case 2:
			data = sys_in16(config->virtio_io_res+offset);

			break;
		case 4:
			data = sys_in32(config->virtio_io_res+offset);

			break;
		default:
			LOG_ERR("Reading unknown size %lu on pci",
				(unsigned long)size);
			return 0;
		}
	} else {
		switch (size) {
		case 1:
			data = sys_read8(config->virtio_io_res+offset);

			break;
		case 2:
			data = sys_read16(config->virtio_io_res+offset);

			break;
		case 4:
			data = sys_read32(config->virtio_io_res+offset);

			break;
		default:
		LOG_ERR("Reading unknown size %lu on pci",
			(unsigned long)size);
		return 0;
		}
	}

	return virtio_htog32(config->virtio_pci_modern, data);
}

static void virtio_pci_write(const virtio_device_t *dev, uintptr_t offset,
					uint32_t data, size_t size)
{
	const virtio_config_t *config = &dev->data;

	data = virtio_gtoh32(config->virtio_pci_modern, data);

	if (config->virtio_io_res_type == VIRTIO_IO) {
		switch (size) {
		case 1:
			sys_out8(data, config->virtio_io_res+offset);
			break;
		case 2:
			sys_out16(data, config->virtio_io_res+offset);
			break;
		case 4:
			sys_out32(data, config->virtio_io_res+offset);
			break;
		}
	} else {
		switch (size) {
		case 1:
			sys_write8(data, config->virtio_io_res+offset);
			break;
		case 2:
			sys_write16(data, config->virtio_io_res+offset);
			break;
		case 4:
			sys_write32(data, config->virtio_io_res+offset);
			break;
		}
	}

}

static void virtio_pci_write_64(virtio_device_t *dev, uintptr_t offset,
					uint64_t data)
{
	virtio_config_t *config = &dev->data;

	uint32_t val0, val1;

	data = virtio_gtoh64(config->virtio_pci_modern, data);
	val0 = data & 0xFFFFFFFF;
	val1 = data >> 32;
	virtio_pci_write(dev, offset, val0, 4);
	virtio_pci_write(dev, offset+4, val1, 4);
}

static uint8_t virtio_pci_get_status(virtio_device_t *dev)
{
	virtio_config_t *config = &dev->data;

	uint8_t status;

	if (config->virtio_pci_modern) {
		status = virtio_pci_read(dev,
					VIRTIO_PCI_COMMON_STATUS, 1);
	} else {
		status = virtio_pci_read_conf(config->virtio_pci_bdf,
						VIRTIO_PCI_STATUS, 1);
	}
	return status;
}

static void virtio_pci_set_status(virtio_device_t *dev, uint8_t status)
{
	if (status != VIRTIO_CONFIG_STATUS_RESET) {
		status |= virtio_pci_get_status(dev);
	}

	if (dev->data.virtio_pci_modern) {
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_STATUS, status, 1);
	} else {
		virtio_pci_write(dev, VIRTIO_PCI_STATUS, status, 1);
	}
}

static int virtio_pci_map_modern_resource(const virtio_device_t *dev,
						virtio_modern_resource_t *res)
{

	const virtio_config_t *config = &dev->data;

	/* TODO: There should probably be a better system to track which bars for
	 *	 the virtio device have been read in. It seems if you map the
	 *	 same bar multiple times, writing can cause a page fault
	 */


	if (res->bar == config->virtio_common.bar) {
		res->io_addr = config->virtio_io_res + res->offset;
		return 0;
	}

	if (!res->io_addr) {
		struct pcie_mbar bar;

		if (!pcie_get_mbar(config->virtio_pci_bdf,
			res->bar, &bar)) {
			LOG_ERR("Unable to find resource mbar");
			return -ENXIO;
		}

		device_map(&res->io_addr, bar.phys_addr,
			bar.size, K_MEM_CACHE_NONE);

		if (!res->io_addr) {
			LOG_ERR("Unable to map bar %d into memory",
				res->bar);
			return -ENXIO;
		}
		res->io_addr += res->offset;
		res->type = VIRTIO_MMIO;
	}

	return 0;
}


/* Negotiate features: */
static uint64_t virtio_pci_read_features(virtio_device_t *dev)
{
	uint64_t features;

	if (dev->data.virtio_pci_modern) {
		virtio_pci_write(dev,
				VIRTIO_PCI_COMMON_DFSELECT, 0, 1);
		features = virtio_pci_read(dev, VIRTIO_PCI_COMMON_DF, 4);

		virtio_pci_write(dev,  VIRTIO_PCI_COMMON_DFSELECT, 1, 1);
		features |= ((uint64_t)virtio_pci_read(dev,
						VIRTIO_PCI_COMMON_DF, 4)) << 32;
	} else {
		features = virtio_pci_read(dev, VIRTIO_PCI_HOST_FEATURES, 4);
	}

	return features;
}

static void virtio_pci_write_features(virtio_device_t *dev, uint64_t features)
{
	if (dev->data.virtio_pci_modern) {
		uint32_t feature0 = features;
		uint32_t feature1 = features >> 32;

		virtio_pci_write(dev, VIRTIO_PCI_COMMON_GFSELECT, 0, 1);
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_GF, feature0, 4);
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_GFSELECT, 1, 1);
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_GF, feature1, 4);
	} else {
		virtio_pci_write(dev, VIRTIO_PCI_GUEST_FEATURES, features, 4);
	}
}

int virtio_pci_neogitate_feature(virtio_device_t *dev,
					uint64_t my_features)
{
	virtio_config_t *config = &dev->data;

	config->virtio_pci_host_features = virtio_pci_read_features(dev);

	if (config->virtio_pci_modern) {
		my_features |= VIRTIO_F_VERSION_1;
	}

	LOG_DBG("Host Features: 0x%llx\n", config->virtio_pci_host_features);
	LOG_DBG("My Features: 0x%llx\n", my_features);

	uint64_t features = config->virtio_pci_host_features & my_features;

	/* filter transport features */
	uint64_t transport, mask;

	transport = (1ULL <<
		(VIRTIO_TRANSPORT_F_END - VIRTIO_TRANSPORT_F_START)) - 1;
	transport <<= VIRTIO_TRANSPORT_F_START;

	mask = -1ULL & ~transport;
	mask |= VIRTIO_RING_F_INDIRECT_DESC;
	mask |= VIRTIO_RING_F_EVENT_IDX;
	mask |= VIRTIO_F_VERSION_1;

	features = features & mask;

	LOG_DBG("Negotiated features: 0x%llx\n", features);

	config->virtio_features = features;

	virtio_pci_write_features(dev, features);

	if (config->virtio_pci_modern) {
		virtio_pci_set_status(dev, VIRTIO_CONFIG_S_FEATURES_OK);
		uint8_t status = virtio_pci_get_status(dev);

		if ((status & VIRTIO_CONFIG_S_FEATURES_OK) == 0) {
			LOG_ERR("Device features were not accepted");
			return -ENXIO;
		}
	}

	return 0;
}

/* Virtqueue methods */

static inline void virtio_pci_select_virtqueue(virtio_device_t *dev, int idx)
{
	virtio_config_t *config = &dev->data;

	LOG_DBG("Selecting Virtqueue %d", idx);
	uint32_t offset = config->virtio_pci_modern ? VIRTIO_PCI_COMMON_Q_SELECT
							: VIRTIO_PCI_QUEUE_SEL;

	virtio_pci_write(dev, offset, idx, 2);
}

static unsigned long virtio_pci_virtqueue_notify_off(virtio_device_t *dev,
							int idx)
{

	virtio_config_t *config = &dev->data;

	if (!config->virtio_pci_modern) {
		return VIRTIO_PCI_QUEUE_NOTIFY;
	}

	virtio_pci_select_virtqueue(dev, idx);
	uint16_t notify_offset = virtio_pci_read(dev,
						VIRTIO_PCI_COMMON_Q_NOFF, 2);

	return notify_offset * config->virtio_notify_offset_mult;
}

static inline uint16_t virtio_pci_virtqueue_size(virtio_device_t *dev,
							int idx)
{
	virtio_config_t *config = &dev->data;

	virtio_pci_select_virtqueue(dev, idx);
	uint32_t offset = config->virtio_pci_modern ? VIRTIO_PCI_COMMON_Q_SIZE
							: VIRTIO_PCI_QUEUE_NUM;

	return virtio_pci_read(dev, offset, 2);
}

static void virtio_pci_disable_virtqueue(virtio_device_t *dev, int idx)
{
	LOG_DBG("Disabling virtqueue %d", idx);
	virtio_pci_select_virtqueue(dev, idx);

	if (dev->data.virtio_pci_modern) {
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_DESCLO, 0);
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_AVAILLO, 0);
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_USEDLO, 0);
	} else {
		virtio_pci_write(dev, VIRTIO_PCI_QUEUE_PFN, 0, 4);
	}
}

static void virtio_pci_set_virtqueue(virtio_device_t *dev, struct virtqueue *vq)
{
	virtio_pci_select_virtqueue(dev, virtqueue_index(vq));

	if (dev->data.virtio_pci_modern) {
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_Q_SIZE,
					virtqueue_size(vq), 2);
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_DESCLO,
					virtqueue_desc_paddr(vq));
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_AVAILLO,
					virtqueue_avail_paddr(vq));
		virtio_pci_write_64(dev, VIRTIO_PCI_COMMON_Q_USEDLO,
					virtqueue_used_paddr(vq));
	} else {
		virtio_pci_write(dev, VIRTIO_PCI_QUEUE_PFN,
		(virtqueue_paddr(vq)) >> VIRTIO_PCI_QUEUE_ADDR_SHIFT,
				4);
	}
}

static void virtio_pci_free_virtqueues(virtio_device_t *dev)
{
	virtio_config_t *config = &dev->data;

	struct vtpci_virtqueue *vqx;

	for (int idx = 0; idx < config->virtio_num_queues; idx++) {
		virtio_pci_disable_virtqueue(dev, idx);

		vqx = &config->virtio_queues[idx];
		virtqueue_free(vqx->vqx_vq);
		vqx->vqx_vq = NULL;
	}

	k_free(config->virtio_queues);
	config->virtio_queues = NULL;
	config->virtio_num_queues = 0;
}

int virtio_pci_alloc_virtqueues(virtio_device_t *dev, int flags, int num_vq,
			struct vq_alloc_info *virtio_info)
{
	int error = 0;
	virtio_config_t *config = &dev->data;

	if (config->virtio_pci_modern) {
		uint16_t max_num_vqs = virtio_pci_read(dev,
						VIRTIO_PCI_COMMON_NUMQ, 2);
		if (num_vq > max_num_vqs) {
			LOG_ERR("Requested PCI virtqueue count %d exceeds max"
				" value of %d", num_vq, max_num_vqs);

			return -E2BIG;
		}
	}

	/* legacy virtqueues must be aligned on a boundary of 4096
	 * modern virtqueues do not have this requirement
	 */
	int align = config->virtio_pci_modern ? 1 : 4096;

	if (config->virtio_num_queues != 0) {
		LOG_ERR("Allocated already %d?\n", config->virtio_num_queues);
		return -EALREADY;
	}
	if (num_vq <= 0) {
		return -EINVAL;
	}

	config->virtio_queues = k_malloc(num_vq * sizeof(struct vtpci_virtqueue));

	if (config->virtio_queues == NULL) {
		LOG_ERR("Not enough memory to allocate virtqueue");
		return -ENOMEM;
	}

	for (int i = 0; i < num_vq; ++i) {
		struct vtpci_virtqueue *vqx = &config->virtio_queues[i];
		struct virtqueue *vq;
		struct vq_alloc_info *vqi = &virtio_info[i];

		uint16_t size = virtio_pci_virtqueue_size(dev, i);
		unsigned long notify = virtio_pci_virtqueue_notify_off(dev, i);

		error = virtqueue_alloc(dev, i, size, notify, align, vqi, &vq);

		if (error) {
			break;
		}

		virtio_pci_set_virtqueue(dev, vq);
		vqx->vqx_vq = *(vqi->vqai_vq) = vq;
		vqx->vqx_no_interrupt = vqi->vqai_intr == NULL;
		config->virtio_num_queues++;
	}

	if (error) {
		LOG_ERR("Error could not allocate virtqueues");
		virtio_pci_free_virtqueues(dev);
	}

	LOG_DBG("Allocated %d virtqueues", num_vq);

	return error;
}

void virtio_pci_notify_virtqueue(virtio_device_t *dev, uint16_t queue,
				unsigned long offset)
{
	LOG_DBG("Notifying virtqueue %d", queue);
	virtio_config_t *config = &dev->data;

	if (config->virtio_pci_modern) {
		if (virtio_pci_map_modern_resource(dev, &config->virtio_notify)) {
			LOG_ERR("Unable to map Notify modern resource");
			return;
		}
		sys_write16(virtio_gtoh16(config->virtio_pci_modern, queue),
			config->virtio_notify.io_addr + offset);
	} else {
		virtio_pci_write(dev, offset, queue, 2);
	}
}


/* Interrupt methods */

static uint8_t virtio_pci_read_isr(const virtio_device_t *dev)
{

	const virtio_config_t *config = &dev->data;

	if (config->virtio_pci_modern) {
		if (!config->virtio_isr.io_addr) {
			return 0;
		}
		return sys_read8(config->virtio_isr.io_addr);
	}

	return virtio_pci_read(dev, VIRTIO_PCI_ISR, 1);
}

/* Handlers */
static void virtio_pci_device_config_change(const void *arg)
{
	const virtio_device_t *dev = arg;

	LOG_DBG("Device config changed");
	virtio_bus_api_t *api = dev->api;

	if (api->config_change != NULL)
		api->config_change(dev);
}

#if defined(CONFIG_PCIE_MSI_X) && defined(CONFIG_PCIE_MSI_MULTI_VECTOR)

static void virtio_pci_shared_msix_intr(const void *arg)
{
	LOG_DBG("Received MSI-X shared interrupt");

	const virtio_device_t *dev = arg;
	const virtio_config_t *config = &dev->data;
	struct vtpci_virtqueue *vqx = config->virtio_queues;

	int no_work = 0;

	for (int i = 0; i < config->virtio_num_queues; ++i, ++vqx) {
		if (!vqx->vqx_no_interrupt)
			no_work |= virtqueue_intr_filter(vqx->vqx_vq);
	}

	if (no_work == 0) {
		LOG_DBG("No work was received on any queues");
		return;
	}

	vqx = config->virtio_queues;
	for (int i = 0; i < config->virtio_num_queues; ++i, ++vqx) {
		if (!vqx->vqx_no_interrupt) {
			virtqueue_disable_intr(vqx->vqx_vq);
			virtqueue_intr(vqx->vqx_vq);
		}
	}
}

static void virtio_pci_pervirtio_msix_intr(const void *arg)
{
	LOG_DBG("Received MSI-X interrupt handler with per-vq");

	const struct vtpci_virtqueue *vqx = arg;

	if (!virtqueue_intr_filter(vqx->vqx_vq)) {
		LOG_DBG("Queue received interrupt with no work to do");
		return;
	}

	virtqueue_intr(vqx->vqx_vq);
}

static int virtio_pci_register_msix_intr(virtio_device_t *dev, int offset,
						int vector)
{
	virtio_pci_write(dev, offset, vector, 2);
	return (virtio_pci_read(dev, offset, 2) == vector ? 0 : -ENODEV);
}

#endif /* CONFIG_PCIE_MSI_X */

static void virtio_pci_legacy_intx(const void *arg)
{
	const virtio_device_t *dev = arg;
	const virtio_config_t *config = &dev->data;

	LOG_DBG("Received legacy interrupt");

	uint8_t isr = virtio_pci_read_isr(dev);

	if (isr & VIRTIO_PCI_ISR_CONFIG) {
		LOG_DBG("Legacy interrupt informed device configuration change");
		virtio_pci_device_config_change(dev);
	}

	if (isr & VIRTIO_PCI_ISR_INTR) {
		LOG_DBG("Legacy interrupt notifies for virtqueue");
		struct vtpci_virtqueue *vqx = config->virtio_queues;

		for (int i = 0; i < config->virtio_num_queues; ++i, ++vqx) {
			if (!vqx->vqx_no_interrupt)
				virtqueue_intr(vqx->vqx_vq);
		}
	}
}

static inline uint32_t _read_pcie_irq_data(pcie_bdf_t bdf)
{
	uint32_t data;

	data = virtio_pci_read_conf(bdf, PCIE_CONF_INTR * 4, 4);

	virtio_pci_write_conf(bdf, PCIE_CONF_INTR * 4,
				data | PCIE_CONF_INTR_IRQ_NONE, 4);

	return data;
}

static inline void _write_pcie_irq_data(pcie_bdf_t bdf, uint32_t data)
{
	virtio_pci_write_conf(bdf, PCIE_CONF_INTR * 4, data, 4);
}

#if defined(CONFIG_PCIE_MSI_X) && defined(CONFIG_PCIE_MSI_MULTI_VECTOR)
static int virtio_pci_msix_pervirtio_setup(virtio_device_t *dev, int priority)
{
	virtio_config_t *config = &dev->data;

	if (config->virtio_flags & VTPCI_FLAG_NO_MSIX) {
		LOG_ERR("MSI-X interrupts are not supported");
		return -ENOTSUP;
	}

	uint8_t n_vectors = 0;

	for (int i = 0; i < config->virtio_num_queues; ++i) {
		if (!config->virtio_queues[i].vqx_no_interrupt)
			n_vectors++;
	}

	/* additional vector for config changes */
	n_vectors += 1;

	LOG_DBG("Requesting %d MSI-X vectors", n_vectors);

	if (n_vectors > config->virtio_msix_count) {
		LOG_ERR("Virtio provides %d vectors, not enough for a per-vq"
			" setup", config->virtio_msix_count);
		return -ENXIO;
	}

	config->virtio_msi_vectors = k_malloc(sizeof(msi_vector_t) *
						n_vectors);

	if (config->virtio_msi_vectors == NULL) {
		LOG_ERR("Not enough memory to allocate %d MSI-X vectors",
				n_vectors);
		return -ENOMEM;
	}

	uint8_t alloc_vec = pcie_msi_vectors_allocate(config->virtio_pci_bdf,
							priority,
							config->virtio_msi_vectors,
							n_vectors);

	if (alloc_vec < n_vectors) {
		LOG_ERR("Could only allocate %u/%u vectors", alloc_vec,
				n_vectors);
		return -ENXIO;
	}

	int error = 0;

	LOG_DBG("Allocated %u MSI-X vectors", n_vectors);

	int vector = 0;

	if (!pcie_msi_vector_connect(config->virtio_pci_bdf,
			&config->virtio_msi_vectors[vector++],
			virtio_pci_device_config_change, config, 0)) {
		LOG_ERR("Unable to connect the device configuration change"
			" vector");
		return -ENXIO;
	}

	for (int i = 0; i < config->virtio_num_queues; ++i) {
		if (config->virtio_queues[i].vqx_no_interrupt)
			continue;
		if (!pcie_msi_vector_connect(config->virtio_pci_bdf,
					&config->virtio_msi_vectors[vector++],
					virtio_pci_pervirtio_msix_intr,
					&config->virtio_queues[i],
					0)) {
			LOG_ERR("Failed to setup MSI-X interrupt handler to"
				" vector %u\n", vector);
			return -ENXIO;
		}

		msi_vector_t *vec = &config->virtio_msi_vectors[i];

		LOG_DBG("MSI-X Vector %u[IRQ=%u,ISRV=%u]", i, vec->arch.irq,
								vec->arch.vector);
	}

	if (!pcie_msi_enable(config->virtio_pci_bdf, config->virtio_msi_vectors,
				n_vectors)) {
		LOG_ERR("Unable to enable MSI-X for virtio");
		return -ENXIO;
	}

	config->virtio_msix_count = n_vectors;

	virtio_pci_register_msix_intr(dev, VIRTIO_PCI_COMMON_MSIX, 0);
	vector = 1;

	for (int i = 0; i < config->virtio_num_queues; i++) {
		uint16_t idx = config->virtio_queues[i].vqx_no_interrupt ?
						VIRTIO_PCI_MSIX_NO_INTERRUPT :
						vector++;

		virtio_pci_select_virtqueue(dev, i);
		error = virtio_pci_register_msix_intr(dev,
					VIRTIO_PCI_COMMON_Q_MSIX, idx);

		if (error) {
			LOG_ERR("Failed to connect vector %d to idx %d",
					i, idx);
			return error;
		}
	}

	config->virtio_flags |= VTPCI_FLAG_MSIX;
	LOG_INF("Enabled MSI-X interrupts\n");
	return 0;
}

static int virtio_pci_msix_shared_setup(virtio_device_t *dev, int priority)
{
	virtio_config_t *config = &dev->data;

	/* require 2 vectors, one for device configuration changes, 1 for
	 * notifying us when a virtqueue has new data
	 */
	uint8_t n_vectors = 2;

	if (config->virtio_flags & VTPCI_FLAG_NO_MSIX) {
		LOG_ERR("No MSI-X support");
		return -ENOTSUP;
	}

	if (n_vectors > config->virtio_msix_count) {
		LOG_ERR("Not enough MSI-X vectors for shared interrupts");
		return -ENXIO;
	}

	config->virtio_msi_vectors = k_malloc(sizeof(msi_vector_t) *
						n_vectors);

	if (config->virtio_msi_vectors == NULL) {
		LOG_ERR("Not enough memory for %u MSI-X vectors", n_vectors);
		return -ENOMEM;
	}

	uint8_t alloc_vec = pcie_msi_vectors_allocate(config->virtio_pci_bdf,
						priority,
						config->virtio_msi_vectors,
						n_vectors);

	if (alloc_vec < n_vectors) {
		LOG_ERR("Could only allocate %u/%u MSI-X vectors", alloc_vec,
				n_vectors);
		return -ENXIO;
	}

	int error = 0;

	LOG_DBG("Allocated %u MSI-X vectors", n_vectors);

	for (int i = 0; i < n_vectors; ++i) {
		void (*handler)(const void *) = i == 0 ?
				virtio_pci_device_config_change :
				virtio_pci_shared_msix_intr;

		if (!pcie_msi_vector_connect(config->virtio_pci_bdf,
						&config->virtio_msi_vectors[i],
						handler, config,
						0)) {
			LOG_ERR("Failed to setup MSI-X interrupt handler to "
				"vector %u\n", i);
			return -ENXIO;
		}

		msi_vector_t *vec = &config->virtio_msi_vectors[i];

		LOG_DBG("MSI-X Vector %u[IRQ=%u,ISRV=%u]", i, vec->arch.irq,
							vec->arch.vector);
	}

	if (!pcie_msi_enable(config->virtio_pci_bdf, config->virtio_msi_vectors,
				n_vectors)){
		LOG_ERR("Unable to enable MSI(-X) for virtio");
		return -ENXIO;
	}

	config->virtio_msix_count = n_vectors;

	virtio_pci_register_msix_intr(dev, VIRTIO_PCI_COMMON_MSIX, 0);

	/* connect all queues requiring an interrupt to vector 1 */
	for (int i = 0; i < config->virtio_num_queues; i++) {
		uint16_t idx = config->virtio_queues[i].vqx_no_interrupt ?
						VIRTIO_PCI_MSIX_NO_INTERRUPT :
						1;

		virtio_pci_select_virtqueue(dev, i);
		error = virtio_pci_register_msix_intr(dev,
					VIRTIO_PCI_COMMON_Q_MSIX, idx);
	}

	config->virtio_flags |= VTPCI_FLAG_SHARED_MSIX | VTPCI_FLAG_MSIX;
	LOG_INF("Enabled MSI-X shared interrupts");
	return 0;
}

#endif /* CONFIG_PCIE_MSI_X */

static int virtio_pci_legacy_intx_setup(virtio_device_t *dev, int priority)
{
	virtio_config_t *config = &dev->data;

	unsigned int dev_irq = pcie_alloc_irq(config->virtio_pci_bdf);

	LOG_DBG("Legacy INTx on IRQ %d", dev_irq);

	if (dev_irq == PCIE_CONF_INTR_IRQ_NONE) {
		LOG_ERR("Unable to create IRQ for INTx interrupts");
		return -ENXIO;
	}

	irq_connect_dynamic(dev_irq, priority, virtio_pci_legacy_intx, config, 0);
	LOG_DBG("Connected IRQ %d to Legacy Virtio Interrupt", dev_irq);

	irq_enable(dev_irq);
	LOG_DBG("Enabled interrupts for IRQ %d", dev_irq);

	config->virtio_flags |= VTPCI_FLAG_INTX;
	LOG_INF("Enabled legacy virtio interrupts");

	if (virtio_pci_map_modern_resource(dev, &config->virtio_isr)) {
		LOG_ERR("Unable to map ISR modern resource");
	} else {
		LOG_DBG("Mapped virtio ISR resource");
	}

	return 0;
}

static void virtio_pci_enable_virtqueues(virtio_device_t *dev)
{
	virtio_config_t *config = &dev->data;

	for (int idx = 0; idx < config->virtio_num_queues; ++idx) {
		virtio_pci_select_virtqueue(dev, idx);
		virtio_pci_write(dev, VIRTIO_PCI_COMMON_Q_ENABLE, 1, 2);
	}
}

int virtio_pci_setup_interrupts(virtio_device_t *dev, int priority, int method)
{
	int error = 0;
	uint32_t key = irq_lock();

	virtio_config_t *config = &dev->data;

#if defined(CONFIG_PCIE_MSI_X) && defined(CONFIG_PCIE_MSI_MULTI_VECTOR)
	if (method == VIRTIO_PCI_MSIX_PERVQ) {
		error = virtio_pci_msix_pervirtio_setup(dev, priority);
	} else if (method == VIRTIO_PCI_MSIX_SHARED) {
		error = virtio_pci_msix_shared_setup(dev, priority);
	} else if (method == VIRTIO_PCI_LEGACY) {
#else
	if (method == VIRTIO_PCI_LEGACY) {
#endif /* CONFIG_PCIE_MSI_X */
		error = virtio_pci_legacy_intx_setup(dev, priority);
	} else {
		LOG_ERR("Unknown setup interrupt method %d", method);
		error = -ENOTSUP;
	}

	if (error) {
		goto fail;
	}

	if (config->virtio_pci_modern) {
		LOG_DBG("Enabling all virtqueues");
		virtio_pci_enable_virtqueues(dev);
	}
fail:
	irq_unlock(key);

	return error;
}

/* Device config utilities */
static int virtio_pci_legacy_read_config(virtio_device_t *dev, uint32_t offset,
						void *out, size_t width)
{
	bool has_msix = (dev->data.virtio_flags & VTPCI_FLAG_MSIX) != 0;

	offset += has_msix ? 24 : 20;

	switch (width) {
	case 1:
		*(uint8_t *)out = virtio_pci_read(dev, offset, width);
		break;
	case 2:
		*(uint16_t *)out = virtio_pci_read(dev, offset, width);
		break;
	case 4:
		*(uint32_t *)out = virtio_pci_read(dev, offset, width);
		break;
	case 8:
		*(uint32_t *)out = virtio_pci_read(dev, offset, width);
		*(((uint32_t *)out)+1) = virtio_pci_read(dev, offset+4,
								width);
		break;
	default:
		LOG_ERR("Unknown width %lu trying to read from device "
			"config", (unsigned long)width);
		return -ENOTSUP;
	}

	return 0;
}

static uint64_t virtio_pci_modern_device_read_64(virtio_device_t *dev,
							uint32_t offset)
{
	int gen;
	uint32_t val0, val1;

	virtio_modern_resource_t *device_res = &dev->data.virtio_device_conf;

	do {
		gen = virtio_pci_read(dev, VIRTIO_PCI_COMMON_CFGGENERATION, 1);
		val0 = sys_read32(device_res->io_addr + offset);
		val1 = sys_read32(device_res->io_addr + offset + 4);
	} while (gen != virtio_pci_read(dev, VIRTIO_PCI_COMMON_CFGGENERATION, 1));

	return (((uint64_t) val1 << 32) | val0);
}

static int virtio_pci_modern_read_config(virtio_device_t *dev, uint32_t off,
						void *out, size_t width)
{
	virtio_modern_resource_t *device_res = &dev->data.virtio_device_conf;

	if (virtio_pci_map_modern_resource(dev, device_res)) {
		LOG_ERR("Unable to map device config modern resource");
		return -ENXIO;
	}

	switch (width) {
	case 1:
		*(uint8_t *)out = sys_read8(device_res->io_addr + off);
		break;
	case 2:
		*(uint16_t *)out = virtio_htog16(true,
				sys_read16(device_res->io_addr + off));
		break;
	case 4:
		*(uint32_t *)out = virtio_htog32(true,
				sys_read32(device_res->io_addr + off));
		break;
	case 8:
		*(uint64_t *)out = virtio_htog64(true,
			virtio_pci_modern_device_read_64(dev, off));
		break;
	default:
		LOG_ERR("Unknown width %lu trying to read from device "
			"config", (unsigned long) width);
		return -ENOTSUP;
	}

	return 0;
}

int virtio_pci_read_config(virtio_device_t *dev, uint32_t offset,
				void *out, size_t width)
{
	if (dev->data.virtio_pci_modern) {
		return virtio_pci_modern_read_config(dev, offset, out, width);
	} else {
		return virtio_pci_legacy_read_config(dev, offset, out, width);
	}
}

static int virtio_pci_legacy_write_config(virtio_device_t *dev, uint32_t offset,
						void *out, size_t width)
{
	bool has_msix = (dev->data.virtio_flags & VTPCI_FLAG_MSIX) != 0;

	offset += has_msix ? 24 : 20;

	switch (width) {
	case 1:
		virtio_pci_write(dev, offset, *(uint8_t *)out, width);
		break;
	case 2:
		virtio_pci_write(dev, offset, *(uint16_t *)out, width);
		break;
	case 4:
		virtio_pci_write(dev, offset, *(uint32_t *)out, width);
		break;
	case 8:
		virtio_pci_write_64(dev, offset, *(uint64_t *)out);
		break;
	default:
		LOG_ERR("Unknown width %lu trying to read from device "
			"config", (unsigned long)width);
		return -ENOTSUP;
	}

	return 0;
}

static int virtio_pci_modern_write_config(virtio_device_t *dev, uint32_t off,
						void *out, size_t width)
{
	virtio_modern_resource_t *device_res = &dev->data.virtio_device_conf;

	if (virtio_pci_map_modern_resource(dev, device_res)) {
		LOG_ERR("Unable to map device config modern resource");
		return -ENXIO;
	}

	switch (width) {
	case 1:
		sys_write8(*(uint8_t *)out, device_res->io_addr + off);
		break;
	case 2:{
		uint16_t val = virtio_gtoh16(true, *(uint16_t *)out);

		sys_write16(val, device_res->io_addr + off);
		break;
	}
	case 4: {
		uint32_t val = virtio_gtoh32(true, *(uint32_t *)out);

		sys_write32(val, device_res->io_addr + off);
		break;
	}
	case 8: {
		uint64_t val = virtio_gtoh64(true, *(uint64_t *)out);

		sys_write32((uint32_t)val, device_res->io_addr + off);
		sys_write32(val >> 32, device_res->io_addr + off + 4);
		break;
	}
	default:
		LOG_ERR("Unknown width %lu trying to read from "
			"device config", (unsigned long)width);
		return -ENOTSUP;
	}

	return 0;
}

int virtio_pci_write_config(virtio_device_t *dev, uint32_t offset,
				void *out, size_t width)
{
	if (dev->data.virtio_pci_modern) {
		return virtio_pci_modern_write_config(dev, offset, out, width);
	} else {
		return virtio_pci_legacy_write_config(dev, offset, out, width);
	}
}

/* PCI virtio device setup functions */

static int virtio_pci_enumerate_caps(virtio_device_t *dev,
					pcie_bdf_t pci_device)
{
	virtio_config_t *config = &dev->data;

	uint32_t header_offset;

	switch (virtio_pci_read_conf(pci_device, 0xE, 1)) {
	case 0x0:
	case 0x1:
		header_offset = VIRTIO_CAP_OFFSET_HEADER_0;
		break;
	case 0x2:
		header_offset = VIRTIO_CAP_OFFSET_HEADER_2;
		break;
	default:
		return -ENXIO;
	}

	uint8_t offset = virtio_pci_read_conf(pci_device, header_offset, 1);
	bool has_msi = false;
	bool has_msix = false;
	uint8_t cfgs = 0;

	while (offset != 0) {
		uint8_t cap_type = virtio_pci_read_conf(pci_device, offset, 1);

		LOG_DBG("Found cap type: %d\n", cap_type);
		if (cap_type == VIRTIO_PCI_SPECIFIC_VENDOR_CAP) {

			if (!config->virtio_pci_modern) {
				LOG_WRN("PCI device 0x%x has caps but is legacy",
						pci_device);
				goto next_offset;
			}

			uint8_t cfg_type = virtio_pci_read_conf(pci_device,
								offset+3, 1);

			LOG_DBG("Virtio Config Type: %x", cfg_type);
			virtio_modern_resource_t *res = NULL;

			switch (cfg_type) {
			case VIRTIO_PCI_CAP_COMMON_CFG:
				res = &config->virtio_common;

				cfgs |= 1 << (cfg_type - 1);
				break;
			case VIRTIO_PCI_CAP_NOTIFY_CFG:
				res = &config->virtio_notify;

				cfgs |= 1 << (cfg_type - 1);
				break;
			case VIRTIO_PCI_CAP_DEVICE_CFG:
				res = &config->virtio_device_conf;

				cfgs |= 1 << (cfg_type - 1);
				break;
			case VIRTIO_PCI_CAP_ISR_CFG:
				res = &config->virtio_isr;

				cfgs |= 1 << (cfg_type - 1);
				break;
			case VIRTIO_PCI_CAP_PCI_CFG:
				break;
			default:
				LOG_WRN("Unknown CFG type: 0x%x",
						cfg_type);
				break;
			}
			if (res == NULL) {
				goto next_offset;
			}

			res->cap_offset = offset;
			res->bar = virtio_pci_read_conf(pci_device, offset+4, 1);
			res->offset = virtio_pci_read_conf(pci_device, offset+8,
								4);
			res->length = virtio_pci_read_conf(pci_device, offset+12,
								4);
			res->type = PCIE_CONF_BAR_IO(virtio_pci_read_conf(
						pci_device, 0x10+(res->bar*4), 4
					)) ? VIRTIO_IO : VIRTIO_MMIO;

			if (cfg_type == VIRTIO_PCI_CAP_NOTIFY_CFG) {
				uint32_t off = virtio_pci_read_conf(pci_device,
								offset+16, 4);

				config->virtio_notify_offset_mult = off;
			}
		} else if (cap_type == VIRTIO_PCI_CAP_MSI) {
			has_msi = true;
		}
#if defined(CONFIG_PCIE_MSI_X) && defined(CONFIG_PCIE_MSI_MULTI_VECTOR)
		else if (cap_type == VIRTIO_PCI_CAP_MSIX) {
			uint16_t count = virtio_pci_read_conf(pci_device,
								offset+2, 4);
			config->virtio_msix_count = (count & 0x3FF) + 1;
			has_msix = true;
		}
#endif /* CONFIG_PCIE_MSI_X */
next_offset:
		offset = virtio_pci_read_conf(pci_device, offset+1, 1);
	}

	if (config->virtio_pci_modern && cfgs != 15) {
		LOG_ERR("CFGS is %d\n", cfgs);
		return -ENXIO;
	}

	if (!has_msi) {
		LOG_DBG("Virtio has no MSI support");
		config->virtio_flags |= VTPCI_FLAG_NO_MSI;
	}

	if (!has_msix) {
		LOG_DBG("Virtio has no MSI-X support");
		config->virtio_flags |= VTPCI_FLAG_NO_MSIX;
	}

	LOG_INF("Enumerated virtio capabilities");
	return 0;
}

static void virtio_pci_reset(virtio_device_t *dev)
{
	virtio_config_t *config = &dev->data;

	virtio_pci_set_status(dev, VIRTIO_CONFIG_STATUS_RESET);
	while (virtio_pci_get_status(dev) != VIRTIO_CONFIG_STATUS_RESET
		&& config->virtio_pci_modern){
		k_cpu_idle();
	}
}

static int virtio_pci_pre_init(virtio_device_t *dev, pcie_bdf_t pci_device)
{
	virtio_config_t *config = &dev->data;
	uint32_t conf_data = virtio_pci_read_conf(pci_device, PCIE_CONF_ID, 4);

	if (PCIE_ID_TO_VEND(conf_data) != VIRTIO_PCI_VENDORID) {
		LOG_ERR("Invalid device vendor ID");
		return -EPROTOTYPE;
	}

	uint32_t device_id = PCIE_ID_TO_DEV(conf_data);

	config->virtio_dev_type = VIRTIO_PCI_DEVICE;
	config->virtio_pci_modern = device_id >= VIRTIO_PCI_DEVICEID_MODERN_MIN;
	config->virtio_pci_bdf = pci_device;

	config->virtio_flags = 0;
	config->virtio_num_queues = 0;

	if (config->virtio_pci_modern) {
		config->virtio_dev_type = device_id -
						VIRTIO_PCI_DEVICEID_MODERN_MIN;
		config->virtio_flags |= VTPCI_FLAG_MODERN;
	} else {
		config->virtio_dev_type = device_id - VIRTIO_PCI_DEVICEID_MIN + 1;
	}

	return 0;
}

static int virtio_pci_init_config(virtio_device_t *dev, pcie_bdf_t pci_device)
{
	virtio_config_t *config = &dev->data;
	uint16_t status = virtio_pci_read_conf(pci_device, 0x6, 2);
	int error = 0;

	if (status & (1 << 4)) {
		LOG_DBG("Device has virtio capabilities");
		error = virtio_pci_enumerate_caps(dev, pci_device);
	}
	/* if there is a modern virtio device without cap lists
	 *  this is a problem
	 */
	else if (config->virtio_pci_modern) {
		uint32_t conf_data = virtio_pci_read_conf(pci_device, PCIE_CONF_ID, 4);

		LOG_ERR("Modern virtio device PCIE device: %d.%x.%d ID: %x:%x"
			" cannot map configs", PCIE_BDF_TO_BUS(pci_device),
			PCIE_BDF_TO_DEV(pci_device),
			PCIE_BDF_TO_FUNC(pci_device),
			PCIE_ID_TO_VEND(conf_data),
			PCIE_ID_TO_DEV(conf_data));

		error = -ENXIO;
	} else {
		config->virtio_flags |= VTPCI_FLAG_NO_MSIX;
		config->virtio_flags |= VTPCI_FLAG_NO_MSI;
	}

	return error;
}

static int virtio_pci_init_io(virtio_device_t *dev, pcie_bdf_t pci_device)
{
	virtio_config_t *config = &dev->data;
	uint8_t bar_to_map;

	if (config->virtio_pci_modern) {
		bar_to_map = config->virtio_common.bar;
	} else {
		bar_to_map = 0;
	}

	uint32_t bar_config = virtio_pci_read_conf(pci_device,
						0x10 + (bar_to_map * 4), 4);
	uint32_t bar_addr = PCIE_CONF_BAR_ADDR(bar_config);

	LOG_DBG("Device uses %s write type",
		PCIE_CONF_BAR_IO(bar_config) ? "IO" : "MMIO");

	if (PCIE_CONF_BAR_IO(bar_config)) {
		config->virtio_io_res = bar_addr;
		config->virtio_io_res_type = VIRTIO_IO;
	} else {
		struct pcie_mbar bar;

		if (!pcie_get_mbar(pci_device, bar_to_map, &bar)) {
			return -ENODEV;
		}

		device_map(&config->virtio_io_res, bar.phys_addr, bar.size,
				K_MEM_CACHE_NONE);
		config->virtio_io_res_type = VIRTIO_MMIO;
	}

	return 0;
}

static virtio_bus_api_t virtio_pci_bus_api = {
		.negotiate_feature = virtio_pci_neogitate_feature,
		.with_feature = virtio_pci_with_feature,
		.alloc_virtqueues = virtio_pci_alloc_virtqueues,
		.setup_interrupts = virtio_pci_setup_interrupts,
		.stop = NULL,
		.reinit = NULL,
		.reinit_complete = NULL,
		.notify_virtqueue = virtio_pci_notify_virtqueue,
		.read_config = virtio_pci_read_config,
		.write_config = virtio_pci_write_config,
		.config_change = NULL
};

int virtio_pci_init(virtio_device_t *dev, pcie_bdf_t pci_device)
{
	dev->api = &virtio_pci_bus_api;
	dev->data.virtio_bus_type = VIRTIO_PCI_DEVICE;

	memset(&dev->data, 0, sizeof(virtio_config_t));
	int error = virtio_pci_pre_init(dev, pci_device);

	LOG_DBG("Finished pre-initialization of virtio device");
	if (error) {
		return error;
	}
	error = virtio_pci_init_config(dev, pci_device);
	LOG_DBG("Mapped virtio device config");
	if (error) {
		return error;
	}

	error = virtio_pci_init_io(dev, pci_device);
	if (error) {
		return error;
	}

	LOG_DBG("Initialized virtio device I/O");
	virtio_pci_reset(dev);
	LOG_DBG("Reset virtio device");
	virtio_pci_set_status(dev, VIRTIO_CONFIG_STATUS_ACK);
	virtio_pci_set_status(dev, VIRTIO_CONFIG_STATUS_DRIVER);
	virtio_pci_set_status(dev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
	LOG_DBG("Sent ack response to virtio device");
	return error;
}

static inline bool virtio_pci_generator_increment(uint16_t *bus, uint8_t *dev,
							uint8_t *func)
{
	++(*func);
	if (*func > MAX_FUNC) {
		*func = 0;
		++(*dev);
	}

	if (*dev > MAX_DEV) {
		*dev = 0;
		++(*bus);
	}

	if (*bus > MAX_BUS) {
		return true;
	}

	return false;
}

virtio_pci_gen_t *virtio_pci_generator_next(virtio_pci_gen_t *gen)
{
	if (gen->continue_from == -1) {
		return NULL;
	}

	uint8_t func = gen->continue_from & 0xFF;
	uint8_t dev = (gen->continue_from >> 8) & 0xFF;
	uint16_t bus = (gen->continue_from >> 16) & 0xFFFF;

	while (1) {
		pcie_bdf_t bdf = PCIE_BDF(bus, dev, func);
		uint32_t conf_data = virtio_pci_read_conf(bdf,
							PCIE_CONF_ID * 4, 4);

		bool out_bounds = virtio_pci_generator_increment(&bus, &dev,
								&func);

		if (conf_data != PCIE_ID_NONE &&
			PCIE_ID_TO_VEND(conf_data) == VIRTIO_PCI_VENDORID) {
			gen->bdf = bdf;
			gen->device_type = PCIE_ID_TO_DEV(conf_data);

			if (gen->device_type >= VIRTIO_PCI_DEVICEID_MODERN_MIN) {
				gen->device_type -=
						VIRTIO_PCI_DEVICEID_MODERN_MIN;
			} else {
				gen->device_type -= VIRTIO_PCI_DEVICEID_MIN + 1;
			}

			if (out_bounds) {
				gen->continue_from = -1;
			} else {
				gen->continue_from = func;
				gen->continue_from |= (dev << 8);
				gen->continue_from |= (bus << 16);
			}
			return gen;
		}

		if (out_bounds) {
			return NULL;
		}
	}
}

void virtio_pci_generator_init(virtio_pci_gen_t *gen)
{
	gen->bdf = -1;
	gen->device_type = -1;
	gen->continue_from = 0;
}
