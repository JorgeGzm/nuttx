/****************************************************************************
 * boards/arm/qemu/qemu-armv7a/scripts/dramboot.ld
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

#include "memory.ld"

/* Provide these to avoid using config files for them */

__uflash_start = ORIGIN(uflash);
__uflash_size = LENGTH(uflash);
__usram_start = ORIGIN(usram);
__usram_size = LENGTH(usram);

/* Provide the kernel boundaries as well */

__kflash_start = ORIGIN(kflash);
__kflash_size = LENGTH(kflash);
__ksram_start = ORIGIN(ksram);
__ksram_size = LENGTH(ksram);
__ksram_end = ORIGIN(ksram) + LENGTH(ksram);


OUTPUT_ARCH(arm)

ENTRY(__start)

SECTIONS
{

  /* where the global variable out-of-bounds detection information located */

#ifdef CONFIG_MM_KASAN_GLOBAL
  .kasan.unused : {
    *(.data..LASANLOC*)
  }
  .kasan.global : {
    KEEP (*(.data..LASAN0))
    KEEP (*(.data.rel.local..LASAN0))
  }
  .kasan.shadows : {
    *(.kasan.shadows)
  }
#endif

  .text : {
      _stext = .;            /* Text section */
      __text_start = .;
      *(.vectors)
      *(.text*)
      *(.fixup)
      *(.gnu.warning)
  } > kflash

  . = ALIGN(4096);

  .init_section : {
      _sinit = ABSOLUTE(.);
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
      _einit = ABSOLUTE(.);
      . = ALIGN(4096);
      _etext = .; /* End_1 of .text */
      _sztext = _etext - _stext;
  } > kflash

  .ARM.extab :
  {
      *(.ARM.extab*)
  } > kflash

  /* .ARM.exidx is sorted, so has to go in its own output section.  */

  PROVIDE_HIDDEN (__exidx_start = .);
  .ARM.exidx :
  {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > kflash
  PROVIDE_HIDDEN (__exidx_end = .);

  .rodata : {
      _srodata = .;          /* Read-only data */
      *(.rodata*)
      *(.data.rel.ro*)
      KEEP(*(SORT(.scattered_array*)));
      . = ALIGN(4096);
      _erodata = .;
  } > kflash

  _eronly = LOADADDR(.data);
  .data : {                    /* Data */
      _sdata = .;
      *(.data*)
      . = ALIGN(8);
      __start_impls = .;
      *(.impls)
      KEEP(*(.impls))
      . = ALIGN(4);
      __stop_impls = .;
      _edata = .;
  } > ksram AT > kflash

  .noinit : {
      _snoinit = ABSOLUTE(.);
      *(.noinit*)
      _enoinit = ABSOLUTE(.);
  } > ksram

  .bss : {                     /* BSS */
       _sbss = .;
       *(.bss*)
       . = ALIGN(4096);
       _ebss = .;
  } > ksram

  /* Sections to be discarded */
  /DISCARD/ : {
       *(.exit.text)
       *(.exit.data)
       *(.exitcall.exit)
  }

  /* Stabs debugging sections.  */
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
}

