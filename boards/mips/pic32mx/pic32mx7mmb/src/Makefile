############################################################################
# boards/mips/pic32mx/pic32mx7mmb/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = pic32_boot.c pic32_bringup.c pic32_leds.c pic32_spi.c pic32_mio283qt2.c

ifeq ($(CONFIG_PIC32MX_USBDEV),y)
CSRCS += pic32_usbdev.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += pic32_appinit.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += pic32_usbmsc.c
endif

ifeq ($(CONFIG_INPUT),y)
CSRCS += pic32_touchscreen.c
endif

include $(TOPDIR)/boards/Board.mk
