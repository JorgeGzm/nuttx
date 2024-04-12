/****************************************************************************
 * boards/arm/stm32h7/linum-stm32h753bi/src/stm32_oneshot.c
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
#include <debug.h>
#include <sys/types.h>
#include <nuttx/timers/timer.h>
#include <nuttx/clock.h>
#include <nuttx/timers/oneshot.h>
#include "linum-stm32h753bi.h"
#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_TESTE      (GPIO_OUTPUT | GPIO_PULLUP | GPIO_SPEED_100MHz | \
                        GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timespec g_ts;
static struct oneshot_lowerhalf_s *g_os_lower = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_callback
 ****************************************************************************/

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg)
{
  static bool status_led = false;

  status_led = !status_led;
  stm32_gpiowrite(GPIO_TESTE, status_led);
  
  ONESHOT_START(g_os_lower, oneshot_callback, NULL, &g_ts);
}

/****************************************************************************
 * Name: board_oneshot_init
 *
 * Description:
 *   Configure the oneshot timer driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int stm32_oneshot_init(int timer, uint16_t resolution)
{
  int ret = OK;
  uint32_t sec;
  uint32_t nsec;
  uint32_t duration_us = 2500;

  sec = duration_us / USEC_PER_SEC;
  nsec = ((duration_us) - (sec * USEC_PER_SEC)) * NSEC_PER_USEC;

  g_ts.tv_sec = (time_t) sec;
  g_ts.tv_nsec = (unsigned long)nsec;

 /* configure rgb leds */
 stm32_configgpio(GPIO_TESTE);

 /* turn off rgb leds */
 stm32_gpiowrite(GPIO_TESTE, 0);

  g_os_lower = oneshot_initialize(timer, resolution);
  if (g_os_lower == NULL)
    {
        syslog(LOG_ERR, "ERROR: oneshot_initialize failed\n");
        return -ENOMEM;
    }

  ONESHOT_START(g_os_lower, oneshot_callback, NULL, &g_ts);

  return ret;
}
