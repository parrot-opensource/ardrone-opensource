obj-$(CONFIG_USB_PARROT6) += dwc_otg.o

dwc_otg-objs	:=	dwc_otg_driver.o dwc_otg_attr.o  \
					dwc_otg_cil.o dwc_otg_cil_intr.o \
					dwc_otg_pcd.o dwc_otg_pcd_intr.o \
					dwc_otg_hcd.o dwc_otg_hcd_intr.o \
					dwc_otg_hcd_queue.o dwc_otg_fiq_handler.o

ifeq ($(CONFIG_USB_PARROT6_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

#EXTRA_CFLAGS += -O0
