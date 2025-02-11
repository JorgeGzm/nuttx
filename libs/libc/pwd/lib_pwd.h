/****************************************************************************
 * libs/libc/pwd/lib_pwd.h
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

#ifndef __LIBS_LIBC_PWD_LIB_PWD_H
#define __LIBS_LIBC_PWD_LIB_PWD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pwd.h>
#include <shadow.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_PASSWD_LINESIZE
#  define CONFIG_LIBC_PASSWD_LINESIZE 80
#endif

#define ROOT_NAME   "root"
#define ROOT_UID    0
#define ROOT_GID    0
#define ROOT_GEOCS  "root"
#define ROOT_DIR    "/root"
#define ROOT_SHELL  "/bin/nsh"
#define ROOT_PASSWD "root"
#define ROOT_PWDP   "$1$123$SGj4CnC7VtiFx.tjjtazK1"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Data for non-reentrant group functions */

EXTERN int g_passwd_index;
EXTERN struct passwd g_passwd;
EXTERN struct spwd g_spwd;
EXTERN char g_passwd_buffer[CONFIG_LIBC_PASSWD_LINESIZE];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct passwd *getpwbuf(uid_t uid, gid_t gid, FAR const char *name,
                            FAR const char *gecos, FAR const char *dir,
                            FAR const char *shell, FAR const char *passwd);
int getpwbuf_r(uid_t uid, gid_t gid, FAR const char *name,
               FAR const char *gecos, FAR const char *dir,
               FAR const char *shell, FAR const char *passwd,
               FAR struct passwd *pwd, FAR char *buf, size_t buflen,
               FAR struct passwd **result);

#ifdef CONFIG_LIBC_PASSWD_FILE
int pwd_findby_name(FAR const char *uname, FAR struct passwd *entry,
                    FAR char *buffer, size_t buflen);
int pwd_findby_uid(uid_t uid, FAR struct passwd *entry, FAR char *buffer,
                   size_t buflen);
int pwd_findby_index(int index, FAR struct passwd *entry,
                     FAR char *buffer, size_t buflen);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __LIBS_LIBC_PWD_LIB_PWD_H */
