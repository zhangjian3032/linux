#bash
pwd=$PWD
cd $1
if [ "$1" == "" ]; then
        exit 0
fi

if [ "$2" == "" ]; then
        echo "No package config : CAM ? BMC"
        exit 1
fi

#Patch replace ...
find $1/. -name  '*.patch' | while read filename; do mv -v "${filename}" "`echo "${filename}" | sed -e 's/\.patch//'`"; done

#BMC
if [ "$2" == "bmc" ]; then
        find $1/. -name  '*.bmc' | while read filename; do mv -v "${filename}" "`echo "${filename}" | sed -e 's/\.bmc//'`"; done
        find $1/. -name  '*.cam' | while read filename; do rm -f "${filename}"; done
	find $1/. -name  '*_cam_*.h' | while read filename; do rm -f "${filename}"; done
	find $1/. -name  '*_cam_*.c' | while read filename; do rm -f "${filename}"; done
	rm -f $1/include/dt-bindings/reset/ast-cam-reset.h
	rm -f $1/arch/arm/configs/ast2500_cam_defconfig
	rm -f $1/drivers/usb/gadget/udc/ast_udc.c
	rm -f $1/drivers/soc/aspeed/ast-jpeg.c
	rm -f $1/arch/arm/configs/ast1220_defconfig
	rm -f $1/arch/arm/configs/ast1220_*_defconfig
	rm -f $1/arch/arm/plat-aspeed/include/plat/regs-cam-scu.h
	rm -f $1/arch/arm/plat-aspeed/ast-cam-scu.c
	rm -rf $1/driver/media/platform/ast-jpeg
	rm -f $1/arch/arm/mach-aspeed/include/mach/regs-cam-scu.h
	rm -f $1/arch/arm/configs/ast2500_dbg_defconfig
	rm -f $1/arch/arm/configs/ast2500_defconfig
	rm -f $1/arch/arm/configs/ast2500_ramfs_defconfig
	rm -f $1/arch/arm/configs/ast2500_squashfs_defconfig
	rm -f $1/arch/arm/configs/ast2400_defconfig
	rm -f $1/arch/arm/boot/dts/ast1220.*
	rm -f $1/arch/arm/boot/dts/ast1220*.*
	rm -f $1/arch/arm/boot/dts/ast2400.dts
	rm -f $1/arch/arm/boot/dts/ast2500.dts
	rm -f $1/drivers/memory/ast-cam-sdmc.c
	rm -f $1/drivers/hwmon/ast_pwm.c
	rm -f $1/drivers/clk/aspeed/clk-cam.c
	rm -f $1/drivers/pinctrl/ast-cam-scu.c
fi

#CAM
if [ "$2" == "cam" ]; then
        find $1/. -name  '*.cam' | while read filename; do mv -v "${filename}" "`echo "${filename}" | sed -e 's/\.cam//'`"; done
        find $1/. -name  '*.bmc' | while read filename; do rm -f "${filename}"; done
	rm -f $1/include/dt-bindings/reset/ast-g4-reset.h
	rm -f $1/include/dt-bindings/reset/ast-g5-reset.h
	rm -f $1/arch/arm/plat-aspeed/ast1070*.*
	rm -f $1/arch/arm/plat-aspeed/include/plat/ast1070*.*
	rm -f $1/arch/arm/boot/dts/ast2400*.dts
	rm -f $1/arch/arm/boot/dts/ast2500*.dts
	rm -f $1/drivers/pci/host/*ast*.c
	rm -f $1/drivers/tty/serial/8250/ast*.c
	rm -f $1/drivers/clk/aspeed/clk-g4.c
	rm -f $1/drivers/clk/aspeed/clk-g5.c
fi

rm -rf $1/arch/arm/plat-aspeed
rm -f $1/arch/arm/configs/ast1520_defconfig
rm -f $1/arch/arm/configs/ast1520_pci_defconfig
rm -f $1/arch/arm/configs/ast2509_defconfig
rm -f $1/arch/arm/configs/ast3200_defconfig
rm -f $1/arch/arm/configs/ast2500_pcie_vga_defconfig
rm -f $1/arch/arm/configs/ast2500fb_defconfig
rm -f $1/arch/arm/mach-aspeed/ast3200.c
rm -f $1/arch/m68k/configs/ast1010_defconfig
rm -rf $1/arch/m68k/include/asm/arch
rm -f $1/arch/m68k/coldfire/ast-timers.c
rm -f $1/arch/m68k/coldfire/config.c
rm -f $1/arch/m68k/coldfire/vic.c
rm -f $1/arch/m68k/include/asm/ast1010_irqs.h
rm -f $1/arch/m68k/include/asm/regs-intr.h
rm -f $1/arch/arm/plat-aspeed/include/plat/regs-cat6613.h
rm -f $1/drivers/clocksource/timer-fttmr010.c
rm -f $1/drivers/video/fbdev/aspeed/astvgafb.c
rm -f $1/drivers/soc/aspeed/adc_cat9883.c
rm -f $1/drivers/soc/aspeed/ast_rfx.c
rm -f $1/drivers/soc/aspeed/ast_vhub.c
rm -f $1/drivers/soc/aspeed/ast_rle.c
rm -f $1/drivers/soc/aspeed/ast_egfx.c
rm -f $1/drivers/soc/aspeed/ast_entropy.c
rm -f $1/drivers/soc/aspeed/ast_formatter.c
rm -f $1/drivers/soc/aspeed/ast_h264.c
rm -f $1/drivers/soc/aspeed/ast_bitblt.c
rm -f $1/drivers/soc/aspeed/ast_bulk.c
rm -f $1/drivers/soc/aspeed/ast_cmdq.c
rm -f $1/drivers/soc/aspeed/ast_hid.c
rm -f $1/drivers/soc/aspeed/ast_mask.c
rm -f $1/drivers/char/sram.c
rm -f $1/drivers/usb/gadget/udc/ast_udc11.c
rm -f $1/drivers/usb/gadget/udc/ast_udc20.c
rm -f $1/drivers/usb/gadget/udc/ast_vhub.c
rm -rf $1/drivers/video/fbdev/aspeed/it66121
rm -f $1/drivers/video/fbdev/aspeed/display-sys.c
rm -f $1/drivers/video/fbdev/aspeed/display-sys.h
rm -f $1/drivers/video/fbdev/aspeed/vga_ddc.c
rm -f $1/drivers/net/ethernet/aspeed/ftgmac100.c
rm -f $1/drivers/net/ethernet/aspeed/ftgmac100.h
rm -f $1/drivers/video/fbdev/vga_ddc.c
rm -f $1/drivers/serial/ast_serial.c
rm -f $1/drivers/soc/aspeed/ast_video.c.ryan
rm -f $1/drivers/soc/aspeed/ast_vgavideo.c
rm -f $1/drivers/video/fbdev/astvgafb.c

rm -f $1/package.sh
