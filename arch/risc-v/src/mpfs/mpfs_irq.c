/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_irq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "riscv_ipi.h"

#include "mpfs.h"
#include "mpfs_plic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable Machine interrupts */

  up_irq_save();

  /* Initialize PLIC for current hart */

  mpfs_plic_init_hart(up_cpu_index());

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
  size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
  riscv_stack_color(g_intstackalloc, intstack_size);
#endif

  /* Set priority for all global interrupts to 1 (lowest) */

  for (int id = 1; id <= NR_IRQS; id++)
    {
      putreg32(MPFS_PLIC_PRIO_MIN, MPFS_PLIC_PRIORITY + (4 * id));
    }

  /* Attach the common interrupt handler */

  riscv_exception_attach();

#ifdef CONFIG_SMP
  /* Clear IPI for CPU0 */

  riscv_ipi_clear(0);

  up_enable_irq(RISCV_IRQ_SOFT);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int extirq = 0;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read m/sstatus & clear machine software interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & clear timer interrupt enable in m/sie */

      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;
      if (extirq < 0 || extirq > NR_IRQS - MPFS_IRQ_EXT_START)
        {
          PANIC();
        }

      mpfs_plic_disable_irq(extirq);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      /* Read m/sstatus & set machine software interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      /* Read m/sstatus & set timer interrupt enable in m/sie */

      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq >= MPFS_IRQ_EXT_START)
    {
      extirq = irq - MPFS_IRQ_EXT_START;
      if (extirq < 0 || extirq > NR_IRQS - MPFS_IRQ_EXT_START)
        {
          PANIC();
        }

      mpfs_plic_clear_and_enable_irq(extirq);
    }
}

/****************************************************************************
 * Name: riscv_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void riscv_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  /* Enable external interrupts (mie/sie) */

  SET_CSR(CSR_IE, IE_EIE);

  /* Read and enable global interrupts (M/SIE) in m/sstatus */

  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);

  return oldstat;
}
