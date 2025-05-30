# ##############################################################################
# sched/mqueue/CMakeLists.txt
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

set(SRCS)

if(NOT CONFIG_DISABLE_MQUEUE OR NOT CONFIG_DISABLE_MQUEUE_SYSV)
  list(APPEND SRCS mq_initialize.c mq_waitirq.c mq_recover.c)
endif()

if(NOT CONFIG_DISABLE_MQUEUE)

  list(
    APPEND
    SRCS
    mq_send.c
    mq_sndinternal.c
    mq_receive.c
    mq_rcvinternal.c
    mq_msgfree.c
    mq_msgqalloc.c
    mq_msgqfree.c
    mq_setattr.c
    mq_notify.c
    mq_getattr.c)

endif()

if(NOT CONFIG_DISABLE_MQUEUE_SYSV)

  list(
    APPEND
    SRCS
    msgctl.c
    msgget.c
    msginternal.c
    msgrcv.c
    msgsnd.c)

endif()
target_sources(sched PRIVATE ${SRCS})
