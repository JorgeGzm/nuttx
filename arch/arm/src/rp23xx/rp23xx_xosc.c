/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_xosc.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "rp23xx_xosc.h"
#include "hardware/rp23xx_xosc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XOSC_STARTUPDELAY_MULT 6
#define XOSC_STARTUPDELAY (BOARD_XOSC_STARTUPDELAY * XOSC_STARTUPDELAY_MULT)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_xosc_init
 *
 * Description:
 *   Initialize Crystal Oscillator (XOSC).
 *
 ****************************************************************************/

void rp23xx_xosc_init(void)
{
  /* Assumes 1-15 MHz input */

  ASSERT(BOARD_XOSC_FREQ <= (15 * MHZ));
  putreg32(RP23XX_XOSC_CTRL_FREQ_RANGE_1_15MHZ, RP23XX_XOSC_CTRL);

  /* Set xosc startup delay */

  uint32_t startup_delay = (((BOARD_XOSC_FREQ / 1000) + 128) / 256) *
                              XOSC_STARTUPDELAY;
  ASSERT(startup_delay < 1 << 13);
  putreg32(startup_delay, RP23XX_XOSC_STARTUP);

  /* Set the enable bit now that we have set freq range and startup delay */

  setbits_reg32(RP23XX_XOSC_CTRL_ENABLE_ENABLE, RP23XX_XOSC_CTRL);

  /* Wait for XOSC to be stable */

  while (!(getreg32(RP23XX_XOSC_STATUS) & RP23XX_XOSC_STATUS_STABLE))
    ;
}
