#
# SHARP configuration
#

obj-y += sharp-soc-dummy.o

obj-$(CONFIG_SHARP_SHDIAG_SMD) += sharp-soc-smd_read.o

CFLAGS_shrlog.o := -DSVERSION_OF_SOFT=\"$(SH_BUILD_ID)\"
obj-$(CONFIG_SHARP_SHLOG) += shrlog.o

obj-$(CONFIG_SHARP_BOOT) += shboot/

obj-$(CONFIG_SHARP_PNP) += pnp/
