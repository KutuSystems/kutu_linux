source /opt/Xilinx/Vivado/2016.4/settings64.sh
source /opt/Xilinx/SDK/2016.4/settings64.sh
export CROSS_COMPILE=arm-xilinx-linux-gnueabi-
export ARCH=arm
make zynq_kutu_defconfig
make uImage LOADADDR=0x00008000
