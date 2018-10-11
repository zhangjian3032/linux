#
# linux/drivers/spichips/Makefile
#
ifdef CONFIG_SFDP
obj-$(CONFIG_MTD_SPI)		+= spimtd.o spiaccess.o spireg.o generic_sfdp.o
else
obj-$(CONFIG_MTD_SPI)		+= spimtd.o spiaccess.o spireg.o generic.o
endif
obj-$(CONFIG_MTD_PILOT_AVL) += pilotavl.o
obj-$(CONFIG_MTD_SPI_AT)   	+= atmel.o
obj-$(CONFIG_MTD_SPI_INTEL_S33) += intels33.o
obj-$(CONFIG_MTD_PILOT4_SPI)	+= pilot4spi_ctlr.o
