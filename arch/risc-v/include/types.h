/****************************************************************************
 * arch/risc-v/include/types.h
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

/* This file should never be included directly but, rather, only indirectly
 * through sys/types.h
 */

#ifndef __ARCH_RISCV_INCLUDE_TYPES_H
#define __ARCH_RISCV_INCLUDE_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* These are the sizes of the standard integer types.  NOTE that these type
 * names have a leading underscore character.  This file will be included
 * (indirectly) by include/stdint.h and typedef'ed to the final name without
 * the underscore character.  This roundabout way of doings things allows
 * the stdint.h to be removed from the include/ directory in the event that
 * the user prefers to use the definitions provided by their toolchain header
 * files
 */

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;

typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

#if defined(CONFIG_ARCH_RV64ILP32) || defined(CONFIG_ARCH_RV32)
typedef signed long        _int32_t;
typedef unsigned long      _uint32_t;

typedef signed long long   _int64_t;
typedef unsigned long long _uint64_t;
#elif defined(CONFIG_ARCH_RV64)
typedef signed int         _int32_t;
typedef unsigned int       _uint32_t;

typedef signed long        _int64_t;
typedef unsigned long      _uint64_t;
#endif
#define __INT64_DEFINED

typedef _int64_t           _intmax_t;
typedef _uint64_t          _uintmax_t;

#if defined(__WCHAR_TYPE__)
typedef __WCHAR_TYPE__     _wchar_t;
#else
typedef int                _wchar_t;
#endif

typedef int                _wint_t;
typedef int                _wctype_t;

/* Use uintreg_t for register-width integers */

#ifdef CONFIG_ARCH_RV32
typedef _int32_t           intreg_t;
typedef _uint32_t          uintreg_t;
#else
typedef _int64_t           intreg_t;
typedef _uint64_t          uintreg_t;
#endif

#ifdef CONFIG_ARCH_RV64
/* A size is 8 bytes */

#if defined(__SIZE_TYPE__)
/* If __SIZE_TYPE__ is defined we define ssize_t based on size_t.
 * We simply change "unsigned" to "signed" for this single definition
 * to make sure ssize_t and size_t only differ by their signedness.
 */

#define unsigned signed
typedef __SIZE_TYPE__      _ssize_t;
#undef unsigned
typedef __SIZE_TYPE__      _size_t;
#else
typedef signed long        _ssize_t;
typedef unsigned long      _size_t;
#endif

/* This is the size of the interrupt state save returned by irqsave().  */

typedef unsigned long long  irqstate_t;
#else
/* A size is 4 bytes */

#if defined(__SIZE_TYPE__)
/* If __SIZE_TYPE__ is defined we define ssize_t based on size_t.
 * We simply change "unsigned" to "signed" for this single definition
 * to make sure ssize_t and size_t only differ by their signedness.
 */

#define unsigned signed
typedef __SIZE_TYPE__      _ssize_t;
#undef unsigned
typedef __SIZE_TYPE__      _size_t;
#elif defined(CONFIG_ARCH_SIZET_LONG)
typedef signed long        _ssize_t;
typedef unsigned long      _size_t;
#else
typedef signed int         _ssize_t;
typedef unsigned int       _size_t;
#endif

/* This is the size of the interrupt state save returned by irqsave().  */

typedef unsigned int       irqstate_t;
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RISCV_INCLUDE_TYPES_H */
