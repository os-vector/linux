#!/bin/bash

set -e

if [[ ! "$(uname -a)" == *"Linux"* ]]; then
	echo "why are you not on linux lmao"
	exit 0
fi

#wget https://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/arm-linux-gnueabi/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi.tar.xz

if [[ ! -d arch ]]; then
	echo "angery"
	exit 0
fi

#if [[ ! -d toolchain/arm ]]; then
#	mkdir toolchain
#	cd toolchain
#	wget https://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/arm-linux-gnueabi/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi.tar.xz
#	tar -xf gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi.tar.xz
#	mv gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabi arm
#	cd ..
#fi

mkdir -p out
mkdir -p v-out

TC="$HOME/projects/vic-toolchain/arm-linux-gnueabi/bin/arm-linux-gnueabi-"

make ARCH=arm CROSS_COMPILE="${TC}" O=out apq8009_defconfig
make ARCH=arm CROSS_COMPILE="${TC}" O=out zImage dtbs modules -j$(nproc)
make ARCH=arm CROSS_COMPILE="${TC}" O=out  INSTALL_MOD_PATH=$(pwd)/v-out modules_install -j$(nproc)
cat out/arch/arm/boot/zImage out/arch/arm/boot/dts/qcom/qcom-apq8009-anki-vector.dtb > v-out/zImage-dtb

echo
echo "your silly kernel is at ./v-out/zImage-dtb"
echo
