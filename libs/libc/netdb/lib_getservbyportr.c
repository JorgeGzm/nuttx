/****************************************************************************
 * libs/libc/netdb/lib_getservbyportr.c
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

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/net/ip.h>

#include <netdb.h>

#include "lib_netdb.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getservbyport_r
 ****************************************************************************/

int getservbyport_r(int port, FAR const char *proto,
                    FAR struct servent *result_buf, FAR char *buf,
                    size_t buflen, FAR struct servent **result)
{
  int protocol;
  int i;

  DEBUGASSERT(result_buf != NULL && result != NULL);

  /* Linux man page says result must be NULL in case of failure. */

  *result = NULL;

  if (proto == NULL)
    {
      protocol = 0;
    }
  else if (strcmp(proto, "tcp") == 0)
    {
      protocol = IPPROTO_TCP;
    }
  else if (strcmp(proto, "udp") == 0)
    {
      protocol = IPPROTO_UDP;
    }
  else
    {
      return EINVAL;
    }

  for (i = 0; g_services_db[i].s_name; i++)
    {
      if (port == g_services_db[i].s_port &&
          (protocol == 0 || protocol == g_services_db[i].s_protocol))
        {
          result_buf->s_name = (FAR char *)g_services_db[i].s_name;
          result_buf->s_aliases = NULL;
          result_buf->s_port = HTONS(port);

          if (g_services_db[i].s_protocol == IPPROTO_TCP)
            {
              result_buf->s_proto = "tcp";
            }
          else
            {
              result_buf->s_proto = "udp";
            }

          *result = result_buf;
          return 0;
        }
    }

  return ENOENT;
}

