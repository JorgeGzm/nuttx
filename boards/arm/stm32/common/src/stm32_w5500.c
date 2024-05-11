/****************************************************************************
 * boards/arm/stm32/common/src/stm32_w5500.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/net/w5500.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity Check *************************************************************/

/* W5500 is on SPI1 */

#ifndef CONFIG_STM32_SPI1
# error "Need CONFIG_STM32_SPI1 in the configuration"
#endif

/* SPI Assumptions **********************************************************/

#define W5500_SPI_PORTNO 1   /* On SPI1 */
#define W5500_DEVNO      0   /* Only one W5500 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_lower_s
{
  const struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t                     handler;  /* W5500 interrupt handler */
  void                      *arg;      /* Argument that accompanies IRQ */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  stm32_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                          void *arg);
static void stm32_enable(const struct w5500_lower_s *lower, bool enable);
static void stm32_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_lower_s g_enclower =
{
  .lower =
  {
    .frequency = 1000000,
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = stm32_attach,
    .enable    = stm32_enable,
    .reset     = stm32_reset,
  },
  .handler = NULL,
  .arg     = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_attach
 *
 * Description:
 *   Attaches the interrupt handler to the GPIO.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   handler - The handler function
 *   arg - Argument to pass to handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int stm32_attach(const struct w5500_lower_s *lower,
                         xcpt_t handler,
                         void *arg)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  priv->handler = handler;
  priv->arg     = arg;

  /* stm32_gpiosetevent() will attach and enable */

  return OK;
}

/****************************************************************************
 * Name: stm32_enable
 *
 * Description:
 *   Enables the W5500 interrupt handler.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   enable - true to enable, false to disable
 *
 ****************************************************************************/

static void stm32_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct stm32_lower_s *priv = (struct stm32_lower_s *)lower;

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      stm32_gpiosetevent(GPIO_W5500_INTR, false, true, true,
                         priv->handler, priv->arg);
    }
  else
    {
      stm32_gpiosetevent(GPIO_W5500_INTR, false, true, true,
                         NULL, NULL);
    }
}

/****************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Brings the W5500 in or out of reset.
 *
 * Input Parameters:
 *   lower - W5500 lower half
 *   reset - true to reset, false to enable
 *
 ****************************************************************************/

static void stm32_reset(const struct w5500_lower_s *lower, bool reset)
{
  stm32_gpiowrite(GPIO_W5500_RESET, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Initializes the SPI and W5500 drivers.
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  stm32_configgpio(GPIO_W5500_INTR);
  stm32_configgpio(GPIO_W5500_RESET);

  /* Reset the device */

  spi = stm32_spibus_initialize(W5500_SPI_PORTNO);
  if (!spi)
    {
      nerr("ERROR: Failed to initialize SPI port %d\n", W5500_SPI_PORTNO);
      return;
    }

  ret = w5500_initialize(spi, &g_enclower.lower, W5500_DEVNO);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d W5500 device %d: %d\n",
           W5500_SPI_PORTNO, W5500_DEVNO, ret);
      return;
    }

  ninfo("Bound SPI port %d to W5500 device %d\n",
        W5500_SPI_PORTNO, W5500_DEVNO);
}

