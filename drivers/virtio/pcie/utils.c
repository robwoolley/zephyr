/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Copyright (c) 2021, Wind River Systems, Inc.*/

#include <drivers/virtio/virtio_zephyr.h>
#include <drivers/virtio/pcie/pcie.h>
#include <drivers/virtio/pcie/utils.h>


#ifdef CONFIG_PCIE_MMIO_CFG
static void virtio_pcie_mm_conf(pcie_bdf_t bdf, size_t offset,
				bool write, uint32_t *data, int size)
{
	for (int i = 0; i < ARRAY_SIZE(bus_segs); i++) {
		int off = PCIE_BDF_TO_BUS(bdf) - bus_segs[i].start_bus;

		if (off >= 0 && off < bus_segs[i].n_buses) {
			bdf = PCIE_BDF(off,
				       PCIE_BDF_TO_DEV(bdf),
				       PCIE_BDF_TO_FUNC(bdf));

			volatile char *addr
				= (void *)&bus_segs[i].mmio[bdf << 4];

			addr += offset;
			if (write) {
				switch (size) {
				case 4:
					__asm("movl %1, %0" : "=m"
						(*(volatile uint32_t *)addr)
						: "a" (*data));
					break;
				case 2:
					__asm("movw %1, %0" : "=m"
						(*(volatile uint16_t *)addr)
						: "a" ((uint16_t)*data));
					break;
				case 1:
					__asm("movb %1, %0" : "=m"
						(*(volatile uint8_t *)addr)
						: "a" ((uint8_t)*data));
					break;
				}
			} else {
				switch (size) {
				case 4:
					__asm("movl %1, %0" : "=a" (*data)
					: "m" (*(volatile uint32_t *)addr));
					break;
				case 2:
					__asm("movzwl %1, %0" : "=a" (*data)
					: "m" (*(volatile uint16_t *)addr));
					break;
				case 1:
					__asm("movzbl %1, %0" : "=a" (*data)
					: "m" (*(volatile uint8_t *)addr));
					break;
				}
			}
		}
	}
}

#endif /* CONFIG_PCIE_MMIO_CFG */

/* Traditional Configuration Mechanism */

#define PCIE_X86_CAP	0xCF8U	/* Configuration Address Port */
#define PCIE_X86_CAP_BDF_MASK	0x00FFFF00U  /* b/d/f bits */
#define PCIE_X86_CAP_EN		0x80000000U  /* enable bit */
#define PCIE_X86_CAP_WORD_MASK	0x3FU  /*  6-bit word index .. */
#define PCIE_X86_CAP_WORD_SHIFT	2U  /* .. is in CAP[7:2] */

#define PCIE_X86_CDP	0xCFCU	/* Configuration Data Port */

static void virtio_pcie_io_conf(pcie_bdf_t bdf, size_t offset,
				bool write, uint32_t *data, int size)
{
	static struct k_spinlock lock;
	k_spinlock_key_t k;

	bdf &= PCIE_X86_CAP_BDF_MASK;
	bdf |= PCIE_X86_CAP_EN;
	bdf |= offset & 0xFF;

	k = k_spin_lock(&lock);
	sys_out32(bdf, PCIE_X86_CAP);

	if (write) {
		if (size == 1)
			sys_out8(*data, PCIE_X86_CDP);
		else if (size == 2)
			sys_out16(*data, PCIE_X86_CDP);
		else if (size == 4)
			sys_out32(*data, PCIE_X86_CDP);
	} else {
		if (size == 1)
			*data = sys_in8(PCIE_X86_CDP);
		else if (size == 2)
			*data = sys_in16(PCIE_X86_CDP);
		else if (size == 4)
			*data = sys_in32(PCIE_X86_CDP);
	}

	sys_out32(0U, PCIE_X86_CAP);
	k_spin_unlock(&lock, k);
}

static inline void virtio_pcie_conf(pcie_bdf_t bdf, size_t offset,
			     bool write, uint32_t *data, int size)
{
#ifdef CONFIG_PCIE_MMIO_CFG
	if (bus_segs[0].mmio == NULL) {
		pcie_mm_init();
	}

	if (do_pcie_mmio_cfg) {
		virtio_pcie_mm_conf(bdf, offset, write, data, size);
	} else
#endif
	{
		virtio_pcie_io_conf(bdf, offset, write, data, size);
	}
}


uint32_t virtio_pci_read_conf(pcie_bdf_t bdf, uint32_t offset, size_t size)
{
	uint32_t data;

	virtio_pcie_conf(bdf, offset, false, &data, size);
	return data;
}

void virtio_pci_write_conf(pcie_bdf_t bdf, uint32_t offset, uint32_t data,
				size_t size)
{
	virtio_pcie_conf(bdf, offset, true, &data, size);
}
