source /opt/Xilinx/Vivado/2017.4/settings64.sh
source /opt/Xilinx/SDK/2017.4/settings64.sh
export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm
make zynq_zybo_defconfig
make uImage LOADADDR=0x00008000
./scripts/dtc/dtc -I dts -O dtb -o devicetree-v1.dtb ./arch/arm/boot/dts/zynq-Zybo-HDMI-v1.dts
./scripts/dtc/dtc -I dts -O dtb -o devicetree-z7.dtb ./arch/arm/boot/dts/zynq-Zybo-HDMI-z7.dts
