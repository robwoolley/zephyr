/*
 * Copyright (c) 2021, Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#ifdef CONFIG_VIRTIO_RNG

#include <drivers/virtio/virtio.h>
#include <logging/log.h>


struct virtio_rng_dev;

#ifndef CONFIG_VIRTIO_RNG_LOG_LEVEL
#define CONFIG_VIRTIO_RNG_LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#endif

int virtio_rng_init(struct virtio_rng_dev *dev, virtio_bus_type_t bus, void *dat);
int virtio_rng_read_data(struct virtio_rng_dev *dev, void *out, size_t *len);

typedef struct virtio_rng_dev {
				virtio_device_t virtio_dev;
				struct virtqueue *rng_vq;
				int (*rng_read)(struct virtio_rng_dev *dev, void *out, size_t *len);
} virtio_rng_dev_t;

#endif
