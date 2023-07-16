# Keep this first line.
TOP = ../..

# If your firmware targets a specific board, specify it here,
# or omit it and provide it on the command line (make BOARD=foo).
BOARD=d-lev-io

TINYUSB_CDC=1
CFG_TUD_ENABLED=1
CFLAGS += -g -DDEBUG

# Leave this line here.
include $(TOP)/make.mk

SRCS += \
  ./app.c \
  ./usb_descriptors.c \
  ./drivers/es901xk.c \

# Finally, leave this line at the bottom of the file.
include $(TOP)/rules.mk
