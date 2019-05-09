#!/bin/sh

c_dir=$PWD

cd $HOME/STM32MPU_workspace/STM32MP15-Ecosystem-v1.0.0/Developer-Package
source SDK/environment-setup-cortexa7t2hf-neon-vfpv4-openstlinux_weston-linux-gnueabi
export KERNEL_SRC_PATH=$HOME/STM32MPU_workspace/STM32MP15-Ecosystem-v1.0.0/Developer-Package/stm32mp1-openstlinux-4.19-thud-mp1-19-02-20/sources/arm-openstlinux_weston-linux-gnueabi/linux-stm32

cd $c_dir

echo setup done
