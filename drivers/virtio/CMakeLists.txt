zephyr_sources(virtio.c virtio_zephyr.c virtqueue.c subr_sglist.c)

zephyr_sources_ifdef(CONFIG_VIRTIO_PCI pcie/utils.c pcie/pcie.c)
zephyr_sources_ifdef(CONFIG_VIRTIO_RNG rng/rng.c)
