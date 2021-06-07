/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2014, Bryan Venteicher <bryanv@FreeBSD.org>
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

#pragma once

#include <zephyr.h>

#ifdef CONFIG_VIRTIO_PCI
#include <drivers/pcie/pcie.h>
#endif

#include <device.h>

#include <logging/log.h>

#include <drivers/virtio/virtio_config.h>
#include <drivers/virtio/virtio_ids.h>
#include <drivers/virtio/virtqueue.h>
#include <drivers/virtio/virtio_endian.h>

#include <sys/byteorder.h>

/* FIXME: This is a very hacky way to inject the virtio carved out memory into
 *	  the "device tree". But since KConfig values cannot be accessed by
 *	  device trees and this stuff gets resolved in macros at compile time
 *	  anyways then when in rome right?
 */

#ifndef CONFIG_VIRTIO_DRAM_BASE
#define DT_N_NODELABEL_virtio_dram_REG_IDX_0_VAL_ADDRESS (1024 * 1024 * 32)
#else
#define DT_N_NODELABEL_virtio_dram_REG_IDX_0_VAL_ADDRESS CONFIG_VIRTIO_DRAM_BASE
#endif

#ifndef CONFIG_VIRTIO_DRAM_SIZE
#define DT_N_NODELABEL_virtio_dram_REG_IDX_0_VAL_SIZE (1024 * 1024 * 64)
#else
#define DT_N_NODELABEL_virtio_dram_REG_IDX_0_VAL_SIZE CONFIG_VIRTIO_DRAM_SIZE
#endif

#if 0
struct __virtio_config__;
typedef struct __virtio_config__ virtio_config_t;
#endif /* 0 */
typedef struct virtio_dev_data virtio_dev_data_t;

struct vq_alloc_info;
struct virtqueue;

typedef struct device *device_t;

#define VIRTIO_PCI_MSIX_PERVQ   0
#define VIRTIO_PCI_MSIX_SHARED  1
#define VIRTIO_PCI_LEGACY       2
#define VIRTIO_MMIO_NORMAL      3

#ifdef CONFIG_VIRTIO_PCI
#include "pcie/pcie.h"
#endif

typedef uintptr_t virtio_io_data_t;

/* ARM doesn't have I/O ports so sys_(in|out) is not defined on ARM build
 * these can be defined with sys_(write|read) to MMIO boundaries
 */

#ifdef CONFIG_ARM

#define sys_out8        sys_write8
#define sys_out16       sys_write16
#define sys_out32       sys_write32

#define sys_in8         sys_read8
#define sys_in16        sys_read16
#define sys_in32        sys_read32

#endif /* CONFIG_ARM */

typedef struct __virtio_bus_interface__ {
	int (*negotiate_feature)(device_t dev, uint64_t features);
	bool (*with_feature)(device_t dev, uint64_t feature);
	int (*alloc_virtqueues)(device_t dev, int flags, int num,
					struct vq_alloc_info *info);
	int (*setup_interrupts)(device_t dev, int prio, int flags);
	void (*stop)(device_t dev);
	int (*reinit)(device_t dev, uint64_t features);
	void (*reinit_complete)(device_t dev);
	void (*notify_virtqueue)(device_t dev, uint16_t vq,
					unsigned long offset);
	int (*read_config)(device_t dev, uint32_t offset, void *dat,
					size_t len);
	int (*write_config)(device_t dev, uint32_t offset, void *dat,
							size_t len);
	void (*config_change)(const device_t dev);
} virtio_bus_api_t;

typedef enum __virtio_bus_type__ {
	VIRTIO_PCI_DEVICE = 0,
	VIRTIO_MMIO_DEVICE = 1
} virtio_bus_type_t;

typedef enum __virtio_mem_type__ {
	VIRTIO_MMIO = 0,
	VIRTIO_IO = 1
} virtio_mem_type_t;

typedef struct __virtio_modern_resource__ {
	uint32_t cap_offset;
	uint8_t bar;
	uint32_t offset;
	uint32_t length;
	virtio_mem_type_t type; /* mmio vs io */
	virtio_io_data_t io_addr;
} virtio_modern_resource_t;



struct virtio_dev_data {
	virtio_bus_type_t virtio_bus_type;
	int virtio_dev_type;

	uint64_t virtio_features;
	uint32_t virtio_flags;

	struct vtpci_virtqueue *virtio_queues;
	uint32_t virtio_num_queues;

	int virtio_interrupt_id;
	void *virtio_int_handler;

	/* In the case of PCI devices this will be a pointer to the pcie_bdf_t
	 * structure which can be used with the Zephyr driver. In the case of
	 * mmio this will be the base address to use for reading and writing
	 */
	virtio_io_data_t virtio_io_res;
	virtio_mem_type_t virtio_io_res_type;

#ifdef CONFIG_VIRTIO_PCI
	/* pci specific fields */
	pcie_bdf_t virtio_pci_bdf;
	uint64_t virtio_pci_host_features;
#ifdef CONFIG_PCIE_MSI_X
	msi_vector_t *virtio_msi_vectors;
	uint16_t virtio_msix_count;
#endif /* CONFIG_PCIE_MSI_X */
	bool virtio_pci_modern;

	/* modern pci specific fields */
	virtio_modern_resource_t virtio_common;

	virtio_modern_resource_t virtio_notify;
	uint32_t virtio_notify_offset_mult;

	virtio_modern_resource_t virtio_isr;
	virtio_modern_resource_t virtio_device_conf;
#endif /* CONFIG_VIRTIO_PCI */
	struct virtqueue *rng_vq;
};

#if 0
typedef struct __virtio_config__ {
	void *addr;
	uint32_t size;
	sys_snode_t next;
} virtio_config_t;

struct __virtio_device__ {
	virtio_config_t data;
	virtio_bus_api_t *api;
};
#endif /* 0 */

/* TODO: Add better memory sync in hw instead of sw */
#define VIRTIO_MEMORY_BARRIER() __asm__ __volatile__("" : : : "memory")

void virtio_describe(const char *msg, uint64_t features);

static inline void virtio_map_config_change(device_t dev,
			void (*handler)(const device_t dev))
{
	virtio_bus_api_t *conf = (virtio_bus_api_t *)dev->api;

	conf->config_change = handler;
}
