/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */



#ifndef E2_AGENT_API_MOSAIC_H
#define E2_AGENT_API_MOSAIC_H

#include "sm/sm_io.h"
#include "common/ngran_types.h"

typedef struct {
  const char *ip;
  uint16_t port;
  const char *sm_dir;
} e2_agent_args_t;

void init_agent_api(int mcc, int mnc, int mnc_digit_len, int nb_id, int cu_du_id, ngran_node_t ran_type, sm_io_ag_t io, e2_agent_args_t const *args);
void stop_agent_api(void);

#endif

