#!/bin/bash

PATH=/home/kerigan/projects/bootimg_tools:$PATH

mkbootimg --cmdline 'earlycon console=ttyMSM0,115200 ro androidboot.hardware=qcom ehci-hcd.park=3 msm_rtb.filter=0x37 lpm_levels.sleep_disabled=1 rootwait fw_devlink.sync_state=timeout iommu.passthrough=1' \
  --kernel v-out/zImage-dtb \
  --ramdisk NONE \
  -o v-out/test.img

#dd if=/dev/zero of=v-out/zeroes.bin count=184304 bs=1

cat /home/kerigan/projects/wire-os/poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/lk2nd-msm8909.img \
  /home/kerigan/projects/wire-os/poky/build/tmp-glibc/deploy/images/apq8009-robot-robot-perf/lk2nd-zeroes.bin \
  v-out/test.img > v-out/final.img

fastboot flash boot_a v-out/final.img
fastboot --set-active=a reboot
