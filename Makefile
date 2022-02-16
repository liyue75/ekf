# name of your application
APPLICATION = riot_ekf3

# If no BOARD is found in the environment, use this default:
BOARD ?= stm32f4discovery

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# gnrc
# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Activate ICMPv6 error messages
#USEMODULE += gnrc_icmpv6_error
# Specify the mandatory networking modules for IPv6 and UDP
#USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_ipv6
USEMODULE += gnrc_udp
# Add a routing protocol
#USEMODULE += gnrc_rpl
#USEMODULE += auto_init_gnrc_rpl
# This application dumps received packets to STDIO using the pktdump module
#USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
#USEMODULE += gnrc_icmpv6_echo
# Add also the shell, some shell commands
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
#USEMODULE += netstats_l2
#USEMODULE += netstats_ipv6
#USEMODULE += netstats_rpl

USEPKG += cmsis-dsp
USEMODULE += printf_float

# w5100
#USEMODULE += w5100
CFLAGS += -DW5100_SPI_MODE
ENC_SPI ?= SPI_DEV\(1\)
ENC_CS ?= GPIO_PIN\(0,12\)
ENC_INT ?= GPIO_PIN\(0,11\)
CFLAGS += -DW5100_PARAM_SPI=$(ENC_SPI)
CFLAGS += -DW5100_PARAM_CS=$(ENC_CS)
CFLAGS += -DW5100_PARAM_EVT=$(ENC_INT)
#CFLAGS += -DW5100_PARAM_SPI_CLK=20000000

# DMA
#FEATURES_REQUIRED += periph_dma

# spi
FEATURES_REQUIRED += periph_spi

# uart
FEATURES_REQUIRED += periph_uart

# notify
    EXTERNAL_MODULE_DIRS += $(CURDIR)/notify
    INCLUDES += -I$(CURDIR)/notify
    USEMODULE += notify

# hal
    EXTERNAL_MODULE_DIRS += $(CURDIR)/hal
    INCLUDES += -I$(CURDIR)/hal
    USEMODULE += hal

# inertial_sensor
    EXTERNAL_MODULE_DIRS += $(CURDIR)/inertial_sensor
    INCLUDES += -I$(CURDIR)/inertial_sensor
    USEMODULE += inertial_sensor

    ## mpu9250
    EXTERNAL_MODULE_DIRS += $(CURDIR)/inertial_sensor/mpu9250_spi
    INCLUDES += -I$(CURDIR)/inertial_sensor/mpu9250_spi
    USEMODULE += mpu9250_spi

# vehicle
    EXTERNAL_MODULE_DIRS += $(CURDIR)/vehicle
    INCLUDES += -I$(CURDIR)/vehicle
    USEMODULE += vehicle

# scheduler
    EXTERNAL_MODULE_DIRS += $(CURDIR)/scheduler
    INCLUDES += -I$(CURDIR)/scheduler
    USEMODULE += scheduler

# fusion_math
    EXTERNAL_MODULE_DIRS += $(CURDIR)/math
    INCLUDES += -I$(CURDIR)/math
    USEMODULE += fusion_math

# per_info
    EXTERNAL_MODULE_DIRS += $(CURDIR)/per_info
    INCLUDES += -I$(CURDIR)/per_info
    USEMODULE += per_info

# filter
    EXTERNAL_MODULE_DIRS += $(CURDIR)/filter
    INCLUDES += -I$(CURDIR)/filter
    USEMODULE += filter

# accel_cal
    EXTERNAL_MODULE_DIRS += $(CURDIR)/accel_cal
    INCLUDES += -I$(CURDIR)/accel_cal
    USEMODULE += accel_cal

# ahrs
    EXTERNAL_MODULE_DIRS += $(CURDIR)/ahrs
    INCLUDES += -I$(CURDIR)/ahrs
    USEMODULE += ahrs

# ekf3
    EXTERNAL_MODULE_DIRS += $(CURDIR)/ekf3
    INCLUDES += -I$(CURDIR)/ekf3
    USEMODULE += ekf3

# gps
    EXTERNAL_MODULE_DIRS += $(CURDIR)/gps
    INCLUDES += -I$(CURDIR)/gps
    USEMODULE += gps

# common
    EXTERNAL_MODULE_DIRS += $(CURDIR)/common
    INCLUDES += -I$(CURDIR)/common
    USEMODULE += common

#microdds
    #CFLAGS += -DPOSIX_SETSOCKOPT
    USEMODULE += posix_sockets
    USEMODULE += sock_udp
    INCLUDES += -I$(CURDIR)/microdds/microdds/include
    INCLUDES += -I$(CURDIR)/microdds/microcdr/include
    INCLUDES += -I$(RIOTBASE)/sys/include

    #core/serialization
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microdds/src/c/core/serialization
    INCLUDES += -I$(CURDIR)/microdds/microdds/src/c/core/serialization
    USEMODULE += ddsser

    #core/session
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microdds/src/c/core/session
    INCLUDES += -I$(CURDIR)/microdds/microdds/src/c/core/session
    USEMODULE += ddsses

    #core/session/stream
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microdds/src/c/core/session/stream
    INCLUDES += -I$(CURDIR)/microdds/microdds/src/c/core/session/stream
    USEMODULE += ddssesstr

    #profile/transport/ip/udp
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microdds/src/c/profile/transport/ip/udp
    INCLUDES += -I$(CURDIR)/microdds/microdds/src/c/profile/transport/ip/udp
    USEMODULE += ddsudp

    #profile/util
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microdds/src/c/util
    INCLUDES += -I$(CURDIR)/microdds/microdds/src/c/util
    USEMODULE += ddsuti

    #cdr/types
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microcdr/src/c/types
    INCLUDES += -I$(CURDIR)/microdds/microcdr/src/c/types
    USEMODULE += cdr_types

    #cdr
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/microcdr/src/c
    INCLUDES += -I$(CURDIR)/microdds/microcdr/src/c
    USEMODULE += cdr_cdr

    #topic
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds/topic
    INCLUDES += -I$(CURDIR)/microdds/topic/
    USEMODULE += microdds_topic

    #microdds
    EXTERNAL_MODULE_DIRS += $(CURDIR)/microdds
    INCLUDES += -I$(CURDIR)/microdds
    USEMODULE += microdds_api



# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Uncomment the following 2 lines to specify static link lokal IPv6 address
# this might be useful for testing, in cases where you cannot or do not want to
# run a shell with ifconfig to get the real link lokal address.
#IPV6_STATIC_LLADDR ?= '"fe80::cafe:cafe:cafe:1"'
#CFLAGS += -DGNRC_IPV6_STATIC_LLADDR=$(IPV6_STATIC_LLADDR)

# Uncomment this to join RPL DODAGs even if DIOs do not contain
# DODAG Configuration Options (see the doc for more info)
# CFLAGS += -DCONFIG_GNRC_RPL_DODAG_CONF_OPTIONAL_ON_JOIN

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

# Set a custom channel if needed
include $(RIOTMAKE)/default-radio-settings.inc.mk
