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

/*! \file PHY/LTE_TRANSPORT/pss.c
* \brief Top-level routines for generating primary synchronization signal (PSS) V8.6 2009-03
* \author F. Kaltenberger, O. Tonelli, R. Knopp
* \date 2011
* \version 0.1
* \company Eurecom
* \email: raymond.knopp@eurecom.fr
* \note
* \warning
*/
/* file: initial_syncSL.c
   purpose: Initial synchronization procedures for Sidelink (SPSS/SSSS/PSBCH detection)
   author: raymond.knopp@eurecom.fr
   date: 13.05.2018
*/

//#include "defs.h"
#include "PHY/defs.h"
#include "PHY/extern.h"

int initial_syncSL(PHY_VARS_UE *ue) {

  int index;
  int64_t psslevel;
  int64_t avglevel;
  int frame,subframe;

  ue->rx_offsetSL = lte_sync_timeSL(ue,
				    &index,
				    &psslevel,
				    &avglevel);
  printf("index %d, psslevel %lld dB avglevel %lld dB => %d sample offset\n",
	 index,dB_fixed(psslevel),dB_fixed(avglevel),ue->rx_offsetSL);
  if (ue->rx_offsetSL >= 0) {
    int32_t sss_metric;
    int32_t phase_max;
    rx_slsss(ue,&sss_metric,&phase_max,index);
    generate_sl_grouphop(ue);
    
    if (rx_psbch(ue) == -1) {
      ue->slbch_errors++;
      return(-1);
    }
    else {
    // send payload to RRC
      LOG_I(PHY,"Synchronization with SyncREF UE found, sending MIB-SL to RRC\n");
      ue_decode_si(ue->Mod_id,
		   0, // CC_id
		   0, // frame
		   0, // eNB_index
		   NULL, // pdu, NULL for MIB-SL
		   0,    // len, 0 for MIB-SL
		   &ue->slss_rx,
		   &frame,
		   &subframe);

      LOG_I(PHY,"RRC returns MIB-SL for frame %d, subframe %d\n");		   
      return(-1);
    }
  }
}
