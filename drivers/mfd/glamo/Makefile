#
# Makefile for the Smedia Glamo driver(s)
#


obj-$(CONFIG_MFD_GLAMO)			+= glamo-core.o glamo-gpio.o
obj-$(CONFIG_MFD_GLAMO_SPI)		+= glamo-spi.o
obj-$(CONFIG_MFD_GLAMO_SPI_GPIO) 	+= glamo-spi-gpio.o

obj-$(CONFIG_MFD_GLAMO_FB)		+= glamo-fb.o
obj-$(CONFIG_MFD_GLAMO_SPI_FB)		+= glamo-lcm-spi.o
obj-$(CONFIG_MFD_GLAMO_MCI)		+= glamo-mci.o
obj-$(CONFIG_MFD_GLAMO_DRM)		+= glamo-drm.o

glamo-drm-objs	:=  glamo-drm-drv.o glamo-cmdq.o glamo-buffer.o \
                    glamo-display.o glamo-kms-fb.o
