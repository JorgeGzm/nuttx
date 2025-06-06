/****************************************************************************
 * boards/arm/moxart/moxa/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_MOXART_MOXA_INCLUDE_BOARD_H
#define __BOARDS_ARM_MOXART_MOXA_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the device is running on a 4MHz internal RC.
 * These definitions will configure clocking
 *
 * MAINOSC:  Frequency = 12MHz (crysta)
 * PLLA: PLL Divider = 1, Multiplier = 14 to generate PLLACK = 168MHz
 * Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 84MHz
 * CPU clock: 84MHz
 */

#define BOARD_32KOSC_FREQUENCY     (32768)
#define BOARD_SCLK_FREQUENCY       (BOARD_32KOSC_FREQUENCY)
#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multiplier = 14
 */

#define BOARD_CKGR_PLLAR_MUL       (13 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  Source = PLLA
 *  Divider = 2
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* Resulting frequencies */

#define BOARD_PLLA_FREQUENCY       (168000000) /* PLLACK:  14 * 12Mhz / 1 */
#define BOARD_MCK_FREQUENCY        (84000000)  /* MCK:     PLLACK / 2 */
#define BOARD_CPU_FREQUENCY        (84000000)  /* CPU:     MCK */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCCK / (2*(CLKDIV+1))
 *   CLKDIV = MCCK / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 84MHz, CLKDIV = 104, MCI_SPEED = 84MHz / 2 * (104+1) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (104 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 2, MCI_SPEED = 84MHz / 2 * (2+1) = 14 MHz */

#define HSMCI_MMCXFR_CLKDIV        (1 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 1, MCI_SPEED = 84MHz / 2 * (1+1) = 21 MHz */

#define HSMCI_SDXFR_CLKDIV         (1 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states
 *
 * FWS MAX FREQUENCY
 *     1.62V 1.8V
 * --- ----- ------
 *  0  17MHz 19MHz
 *  1  45MHz 50MHz
 *  2  58MHz 64MHz
 *  3  70MHz 80MHz
 *  4  78MHz 90MHz
 */

#define BOARD_FWS                  4

/* LED definitions **********************************************************/

/*  There are three user-controllable LEDs on board the Moxa board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    PB27
 *     TX  Yellow LED   PA21
 *     RX  Yellow LED   PC30
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_L       0
#define BOARD_LED_RX      1
#define BOARD_LED_TX      2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_L_BIT   (1 << BOARD_LED_L)
#define BOARD_LED_RX_BIT  (1 << BOARD_LED_RX)
#define BOARD_LED_TX_BIT  (1 << BOARD_LED_TX)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *  SYMBOL                  MEANING                      LED STATE
 *                                                      L    TX   RX
 *  ----------------------  ------------------------- ----- ---- ----
 */

#define LED_STARTED       0 /* NuttX has been started  OFF  OFF   OFF      */
#define LED_HEAPALLOCATE  0 /* Heap has been allocated OFF  OFF   OFF      */
#define LED_IRQSENABLED   0 /* Interrupts enabled      OFF  OFF   OFF      */
#define LED_STACKCREATED  1 /* Idle stack created      ON   OFF   OFF      */
#define LED_INIRQ         2 /* In an interrupt         N/C  GLOW  OFF      */
#define LED_SIGNAL        2 /* In a signal handler     N/C  GLOW  OFF      */
#define LED_ASSERTION     2 /* An assertion failed     N/C  GLOW  OFF      */
#define LED_PANIC         3 /* The system has crashed  N/C  N/C   Blinking */
#define LED_PANIC         3 /* MCU is is sleep mode    ---- Not used ----  */

#undef CONFIG_SUPPRESS_SERIAL_INTS

/* Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interrupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

/* Button definitions *******************************************************/

/*   There are no buttons on the Moxa board. */

/* GPIO pin configurations **************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

inline void ftintc010_set_trig_mode(int irq, int mode);
inline void ftintc010_set_trig_level(int irq, int level);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_MOXART_MOXA_INCLUDE_BOARD_H */
