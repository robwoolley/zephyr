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

#pragma once

#include <drivers/virtio/virtqueue.h>
#include <drivers/virtio/pcie/virtio_pci_legacy_var.h>
#include <drivers/virtio/pcie/virtio_pci_modern_var.h>
#include <drivers/virtio/pcie/utils.h>
#include <drivers/virtio/virtio.h>

struct __virtio_config__;

typedef struct __virtio_config__ virtio_config_t;

struct vq_alloc_info;
struct virtqueue;

#define VTPCI_FLAG_NO_MSI		0x0001
#define VTPCI_FLAG_NO_MSIX		0x0002
#define VTPCI_FLAG_MODERN		0x0004
#define VTPCI_FLAG_INTX			0x1000
#define VTPCI_FLAG_MSI			0x2000
#define VTPCI_FLAG_MSIX			0x4000
#define VTPCI_FLAG_SHARED_MSIX		0x8000
#define VTPCI_FLAG_ITYPE_MASK		0xF000

#define VIRTIO_PCI_CAP_MSI      0x05
#define VIRTIO_PCI_SPECIFIC_VENDOR_CAP 0x9
#define VIRTIO_PCI_CAP_MSIX     0x11

#define VIRTIO_PCI_MSIX_NO_INTERRUPT 0xFFFF

typedef struct __virtio_pci_interrupt__ {
	int vtpci_interrupt_id;
	void *vtcpi_interrupt_handler;
} virtio_pci_interrupt_t;


/* common functions */
bool virtio_pci_with_feature(virtio_device_t *dev, uint64_t features);
int virtio_pci_neogitate_feature(virtio_device_t *dev, uint64_t features);
int virtio_pci_alloc_virtqueues(virtio_device_t *dev, int flags, int nvqs,
					struct vq_alloc_info *info);
int virtio_pci_setup_interrupts(virtio_device_t *dev, int priority, int method);
void virtio_pci_notify_virtqueue(virtio_device_t *dev, uint16_t queue,
					unsigned long bus_offset);
int virtio_pci_read_config(virtio_device_t *dev, uint32_t offset, void *dat,
				size_t len);
int virtio_pci_write_config(virtio_device_t *dev, uint32_t offset, void *dat,
				size_t len);

/* TODO: Finish implementing the following functions */
void virtio_pci_stop(virtio_device_t *dev);
int virtio_pci_reinit(virtio_device_t *dev, uint64_t features);
void virtio_pci_reinit_complete(virtio_device_t *dev);

struct vtpci_virtqueue {
	struct virtqueue    *vqx_vq;
	bool            vqx_no_interrupt;
	uint32_t        vqx_notify_offset;
};

/* common utils */
typedef struct __virtio_pci_gen__ {
	pcie_bdf_t bdf;
	uint32_t device_type;
	uint32_t continue_from;
} virtio_pci_gen_t;

void virtio_pci_generator_init(virtio_pci_gen_t *gen);
virtio_pci_gen_t *virtio_pci_generator_next(virtio_pci_gen_t *gen);

#define VIRTIO_PCI_FOREACH(_gen_name_p)\
	virtio_pci_gen_t _gen_name_p ## s;\
	virtio_pci_gen_t *_gen_name_p = &_gen_name_p ## s;\
	virtio_pci_generator_init(_gen_name_p);\
	while ((_gen_name_p = virtio_pci_generator_next(_gen_name_p)))


int virtio_pci_init(virtio_device_t *dev, pcie_bdf_t pci_device);
void virtio_test_interrupt(virtio_device_t *dev, uint16_t vector);
