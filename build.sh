#!/bin/bash
mkdir -p output 
cp -f ./arch/arm/configs/muse6735_m1v_m_defconfig  ./output/.config
make -j8 ARCH=arm KBUILD_OUTPUT=./output oldnoconfig
make -j8 ARCH=arm KBUILD_OUTPUT=./output
cp -f ./output/arch/arm/boot/zImage ./arch/arm/boot/
cp -f ./output/arch/arm/boot/zImage-dtb ./arch/arm/boot/zImage-dtb
cp -f ./output/arch/arm/boot/Image ./arch/arm/boot/


