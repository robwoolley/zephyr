zephyr_sources(virtio.c virtqueue.c)

zephyr_sources_ifdef(CONFIG_VIRTIO_PCI pcie/utils.c pcie/pcie.c)
zephyr_sources_ifdef(CONFIG_VIRTIO_RNG rng/rng.c)