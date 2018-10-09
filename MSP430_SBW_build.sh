source /opt/Xilinx/Vivado/2018.2/settings64.sh
source /opt/Xilinx/SDK/2018.2/settings64.sh
export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm
make zynq_MSP430_SBW_defconfig
make uImage LOADADDR=0x00008000
./scripts/dtc/dtc -I dts -O dtb -o devicetree.dtb ./arch/arm/boot/dts/zynq-MSP430-SBW.dts
