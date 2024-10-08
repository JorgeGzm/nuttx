/****************************************************************************
 * graphics/nxglib/lcd/nxglib_filltrapezoid.c
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
#include <fixedmath.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"
#include "nxglib_fillrun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp
 *
 * Description:
 *   Fill a trapezoidal region in the LCD memory with a fixed color.
 *   Clip the trapezoid to lie within a boundng box.  This is useful for
 *   drawing complex shapes that can be broken into a set of trapezoids.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_filltrapezoid, NXGLIB_SUFFIX)
(
  FAR struct lcd_planeinfo_s *pinfo,
  FAR const struct nxgl_trapezoid_s *trap,
  FAR const struct nxgl_rect_s *bounds,
  NXGL_PIXEL_T color)
{
  unsigned int ncols;
  unsigned int topy;
  unsigned int boty;
  unsigned int row;
  unsigned int botw;
  b16_t        topx1;
  b16_t        topx2;
  b16_t        botx1;
  b16_t        botx2;
  b16_t        dx1dy;
  b16_t        dx2dy;
  int          dy;
  int          ix1;
  int          ix2;

  /* Get the top run endpoints */

  topx1 = trap->top.x1;
  topx2 = trap->top.x2;

  /* Calculate the number of rows to render */

  topy  = trap->top.y;
  boty  = trap->bot.y;

  /* Get the bottom run endpoints */

  botx1  = trap->bot.x1;
  botx2  = trap->bot.x2;

  /* Calculate the slope of the left and right side of the trapezoid */

  dy = boty - topy;
  if (dy > 0)
    {
      dx1dy  = b16divi((botx1 - topx1), dy);
      dx2dy  = b16divi((botx2 - topx2), dy);
    }
  else
    {
      /* The trapezoid is a run! Use the average width. */

      topx1  = (topx1 + botx1) >> 1;
      topx2  = (topx2 + botx2) >> 1;
      botx1  = topx1;
      botx2  = topx2;
      dx1dy  = 0;
      dx2dy  = 0;
    }

  /* Perform vertical clipping */

  if (topy < bounds->pt1.y)
    {
      /* Is the entire trapezoid "above" the clipping window? */

      if (boty < bounds->pt1.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Calculate the x values for the new top run */

      dy      = bounds->pt1.y - topy;
      topx1  += dy * dx1dy;
      topx2  += dy * dx2dy;

      /* Clip the top row to render */

      topy    = bounds->pt1.y;
    }

  if (boty > bounds->pt2.y)
    {
      /* Is the entire trapezoid "below" the clipping window? */

      if (topy > bounds->pt2.y)
        {
          /* Yes.. then do nothing */

          return;
        }

      /* Calculate the x values for the new bottom run */

      dy      = boty - bounds->pt2.y;
      botx1  -= dy * dx1dy;
      botx2  -= dy * dx2dy;

      /* Clip the bottom row to render */

      boty    = bounds->pt2.y;
    }

  /* Handle the special case where the sides cross (as in an hourglass) */

  if (botx1 > botx2)
    {
      b16_t tmp;
      ngl_swap(botx1, botx2, tmp);
    }

  /* Fill the run buffer for the maximum run that we will need */

  ix1    = b16toi(topx1);
  ix1    = ngl_clipl(ix1, bounds->pt1.x);
  ix2    = b16toi(topx2);
  ix2    = ngl_clipr(ix2, bounds->pt2.x);
  ncols  = ix2 - ix1 + 1;

  ix1    = b16toi(botx1);
  ix1    = ngl_clipl(ix1, bounds->pt1.x);
  ix2    = b16toi(botx2);
  ix2    = ngl_clipr(ix2, bounds->pt2.x);
  botw   = ix2 - ix1 + 1;

  if (ncols < botw)
    {
      ncols = botw;
    }

  NXGL_FUNCNAME(nxgl_fillrun, NXGLIB_SUFFIX)((NXGLIB_RUNTYPE *)pinfo->buffer,
                                                               color,
                                                               ncols);

  /* Then fill the trapezoid row-by-row */

  for (row = topy; row <= boty; row++)
    {
      /* Handle the special case where the sides cross (as in an hourglass) */

      if (topx1 > topx2)
        {
          b16_t tmp;
          ngl_swap(topx1, topx2, tmp);
          ngl_swap(dx1dy, dx2dy, tmp);
        }

      /* Convert the positions to integer and get the run width, clipping to
       * fit within the bounding box.
       */

      ix1 = b16toi(topx1);
      ix1 = ngl_clipl(ix1, bounds->pt1.x);
      ix2 = b16toi(topx2);
      ix2 = ngl_clipr(ix2, bounds->pt2.x);

      /* Handle some corner cases where we draw nothing.  Otherwise, we will
       * always draw at least one pixel.
       */

      if (ix1 <= ix2)
        {
          /* Then draw the run from ix1 to ix2 at row */

          ncols = ix2 - ix1 + 1;
          pinfo->putrun(pinfo->dev, row, ix1, pinfo->buffer, ncols);
        }

      /* Add the dx/dy value to get the run positions on the next row */

      topx1 += dx1dy;
      topx2 += dx2dy;
    }
}
