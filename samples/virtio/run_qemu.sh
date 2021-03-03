#!/bin/sh

qemu-system-x86_64 -s -D /tmp/qemu.log -m 256M -no-reboot -nographic -net none \
 -pidfile qemu.pid -chardev stdio,id=con,mux=on -serial chardev:con \
 -mon chardev=con,mode=readline -kernel build/zephyr/zephyr.elf \
 -object rng-random,id=rng0 \
 -device virtio-rng-pci,rng=rng0,disable-legacy=off,disable-modern=on
