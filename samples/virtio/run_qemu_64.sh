#!/bin/sh

objcopy -O elf32-i386 build/zephyr/zephyr.elf build/zephyr/zephyr-qemu.elf
objcopy -j .locore build/zephyr/zephyr-qemu.elf build/zephyr/zephyr-qemu-locore.elf
objcopy -R .locore build/zephyr/zephyr-qemu.elf build/zephyr/zephyr-qemu-main.elf

qemu-system-x86_64 -s -D /tmp/qemu.log -m 256M -no-reboot -nographic -net none\
 -pidfile qemu.pid -chardev stdio,id=con,mux=on -serial chardev:con \
 -mon chardev=con,mode=readline \
 -device loader,file=build/zephyr/zephyr-qemu-main.elf -smp cpus=2 \
 -kernel build/zephyr/zephyr-qemu-locore.elf \
 -object rng-random,id=rng0 -device virtio-rng-pci,rng=rng0,disable-legacy=off,disable-modern=on
