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
#ifndef _SS_GNB_CONTEXT_
#define _SS_GNB_CONTEXT_

#include "SidlCommon.h"

typedef struct SSConfigContext_s {
  uint16_t dl_earfcn;
  uint16_t ul_earfcn;
  uint32_t dl_freq;
  uint32_t ul_freq;
  uint16_t curr_attn;
  uint16_t cellId;
  int16_t maxRefPower;
  //TO DO: Need to remove one of the following cellId
  EUTRA_CellId_Type ttcn_cell_id;
  uint16_t eutra_cellId;
  uint16_t nr_cellId;

  /** TODO: To add more */
} SSConfigContext_t;

#endif /* _SS_GNB_CONTEXT_ */

