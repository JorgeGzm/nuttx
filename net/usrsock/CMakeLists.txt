# ##############################################################################
# net/usrsock/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# User Socket source files

if(CONFIG_NET_USRSOCK)
  target_sources(
    net
    PRIVATE usrsock_close.c
            usrsock_conn.c
            usrsock_bind.c
            usrsock_connect.c
            usrsock_getpeername.c
            usrsock_devif.c
            usrsock_shutdown.c
            usrsock_event.c
            usrsock_getsockname.c
            usrsock_getsockopt.c
            usrsock_poll.c
            usrsock_recvmsg.c
            usrsock_sendmsg.c
            usrsock_setsockopt.c
            usrsock_socket.c
            usrsock_sockif.c
            usrsock_accept.c
            usrsock_listen.c
            usrsock_ioctl.c)
endif()
