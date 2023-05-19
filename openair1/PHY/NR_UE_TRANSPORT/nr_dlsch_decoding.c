/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.0  (the "License"); you may not use this file
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

/*! \file PHY/NR_UE_TRANSPORT/nr_dlsch_decoding.c
* \brief Top-level routines for decoding  Turbo-coded (DLSCH) transport channels from 36-212, V8.6 2009-03
* \author R. Knopp
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr
* \note
* \warning
*/

#include "common/utils/LOG/vcd_signal_dumper.h"
#include "PHY/defs_nr_UE.h"
#include "SCHED_NR_UE/harq_nr.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/CODING/coding_extern.h"
#include "PHY/CODING/coding_defs.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "PHY/NR_TRANSPORT/nr_dlsch.h"
#include "SCHED_NR_UE/defs.h"
#include "SIMULATION/TOOLS/sim.h"
#include "executables/nr-uesoftmodem.h"
#include "PHY/CODING/nrLDPC_extern.h"
#include "common/utils/nr/nr_common.h"
#include "openair1/PHY/TOOLS/phy_scope_interface.h"

//#define ENABLE_PHY_PAYLOAD_DEBUG 1

#define OAI_UL_LDPC_MAX_NUM_LLR 27000//26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX = 68*384
//#define OAI_LDPC_MAX_NUM_LLR 27000//26112 // NR_LDPC_NCOL_BG1*NR_LDPC_ZMAX

static extended_kpi_ue kpiStructure = {0};

extended_kpi_ue* getKPIUE(void) {
  return &kpiStructure;
}

void nr_ue_dlsch_init(NR_UE_DLSCH_t dlsch_list[2], int num_dlsch, uint8_t max_ldpc_iterations) {
  for (int i=0; i < num_dlsch; i++) {
    NR_UE_DLSCH_t *dlsch = dlsch_list + i;
    memset(dlsch, 0, sizeof(NR_UE_DLSCH_t));
    dlsch->max_ldpc_iterations = max_ldpc_iterations;
  }
}

void nr_dlsch_unscrambling(int16_t *llr, uint32_t size, uint8_t q, uint32_t Nid, uint32_t n_RNTI)
{
  nr_codeword_unscrambling(llr, size, q, Nid, n_RNTI);
}

bool nr_ue_postDecode(PHY_VARS_NR_UE *phy_vars_ue, notifiedFIFO_elt_t *req, bool last, notifiedFIFO_t *nf_p, int b_size, uint8_t b[b_size], int *num_seg_ok, UE_nr_rxtx_proc_t *proc) {
  ldpcDecode_ue_t *rdata = (ldpcDecode_ue_t*) NotifiedFifoData(req);
  NR_DL_UE_HARQ_t *harq_process = rdata->harq_process;
  NR_UE_DLSCH_t *dlsch = (NR_UE_DLSCH_t *) rdata->dlsch;
  int r = rdata->segment_r;

  merge_meas(&phy_vars_ue->dlsch_deinterleaving_stats, &rdata->ts_deinterleave);
  merge_meas(&phy_vars_ue->dlsch_rate_unmatching_stats, &rdata->ts_rate_unmatch);
  merge_meas(&phy_vars_ue->dlsch_ldpc_decoding_stats, &rdata->ts_ldpc_decode);

  bool decodeSuccess = (rdata->decodeIterations < (1+dlsch->max_ldpc_iterations));

  if (decodeSuccess) {
    memcpy(b+rdata->offset,
           harq_process->c[r],
           rdata->Kr_bytes - (harq_process->F>>3) -((harq_process->C>1)?3:0));

    (*num_seg_ok)++;
  } else {
    if ( !last ) {
      int nb=abortTpoolJob(&get_nrUE_params()->Tpool, req->key);
      nb+=abortNotifiedFIFOJob(nf_p, req->key);
      LOG_D(PHY,"downlink segment error %d/%d, aborted %d segments\n",rdata->segment_r,rdata->nbSegments, nb);
      LOG_D(PHY, "DLSCH %d in error\n",rdata->dlsch_id);
      last = true;
    }
  }

  // if all segments are done
  if (last) {
    kpiStructure.nb_total++;
    kpiStructure.blockSize = dlsch->dlsch_config.TBS;
    kpiStructure.dl_mcs = dlsch->dlsch_config.mcs;
    kpiStructure.nofRBs = dlsch->dlsch_config.number_rbs;

    if (*num_seg_ok == harq_process->C) {
      if (harq_process->C > 1) {
        /* check global CRC */
        int A = dlsch->dlsch_config.TBS;
        int crc_length = A > 3824 ? 3 : 2;
        int crc_type   = A > 3824 ? CRC24_A : CRC16;
        if (!check_crc(b, A + crc_length*8, 0 /* F - unused */, crc_type)) {
          harq_process->ack = 0;
          dlsch->last_iteration_cnt = dlsch->max_ldpc_iterations + 1;
          LOG_E(PHY, " Frame %d.%d LDPC global CRC fails, but individual LDPC CRC succeeded. %d segs\n", proc->frame_rx, proc->nr_slot_rx, harq_process->C);
          LOG_D(PHY, "DLSCH received nok \n");
          return true; //stop
        }
      }
      //LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for nr_slot_rx %d TBS %d mcs %d nb_rb %d harq_process->round %d\n",
      //      phy_vars_ue->Mod_id,nr_slot_rx,harq_process->TBS,harq_process->mcs,harq_process->nb_rb, harq_process->round);
      harq_process->status = SCH_IDLE;
      harq_process->ack = 1;

      //LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for SFN/SF %d/%d (pid %d, status %d, round %d, TBS %d, mcs %d)\n",
      //  phy_vars_ue->Mod_id, frame, subframe, harq_pid, harq_process->status, harq_process->round,harq_process->TBS,harq_process->mcs);

      //if(is_crnti) {
      //  LOG_D(PHY,"[UE %d] DLSCH: Setting ACK for nr_slot_rx %d (pid %d, round %d, TBS %d)\n",phy_vars_ue->Mod_id,nr_slot_rx,harq_pid,harq_process->round,harq_process->TBS);
      //}
      dlsch->last_iteration_cnt = rdata->decodeIterations;
      LOG_D(PHY, "DLSCH received ok \n");
    } else {
      kpiStructure.nb_nack++;
      //LOG_D(PHY,"[UE %d] DLSCH: Setting NAK for SFN/SF %d/%d (pid %d, status %d, round %d, TBS %d, mcs %d) Kr %d r %d harq_process->round %d\n",
      //      phy_vars_ue->Mod_id, frame, nr_slot_rx, harq_pid,harq_process->status, harq_process->round,harq_process->TBS,harq_process->mcs,Kr,r,harq_process->round);
      harq_process->ack = 0;

      //if(is_crnti) {
      //  LOG_D(PHY,"[UE %d] DLSCH: Setting NACK for nr_slot_rx %d (pid %d, pid status %d, round %d/Max %d, TBS %d)\n",
      //        phy_vars_ue->Mod_id,nr_slot_rx,harq_pid,harq_process->status,harq_process->round,dlsch->Mdlharq,harq_process->TBS);
      //}
      dlsch->last_iteration_cnt = dlsch->max_ldpc_iterations + 1;
      LOG_D(PHY, "DLSCH received nok \n");
    }
    return true; //stop
  }
  else
  {
	return false; //not last one
  }
}

void nr_processDLSegment(void* arg) {
  ldpcDecode_ue_t *rdata = (ldpcDecode_ue_t*) arg;
  NR_UE_DLSCH_t *dlsch = rdata->dlsch;
  NR_DL_UE_HARQ_t *harq_process= rdata->harq_process;
  t_nrLDPC_dec_params *p_decoderParms = &rdata->decoderParms;
  rdata->decodeIterations = dlsch->max_ldpc_iterations + 1;

  const int Kr = harq_process->K;
  start_meas(&rdata->ts_deinterleave);

  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_DEINTERLEAVING, VCD_FUNCTION_IN);
  const int E = rdata->E;
  const int Qm = rdata->Qm;
  const int r_offset = rdata->r_offset;
  int16_t w[E];
  short* dlsch_llr = rdata->dlsch_llr;
  nr_deinterleaving_ldpc(E,
                         Qm,
                         w, // [hna] w is e
                         dlsch_llr+r_offset);
  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_DEINTERLEAVING, VCD_FUNCTION_OUT);
  stop_meas(&rdata->ts_deinterleave);

  start_meas(&rdata->ts_rate_unmatch);
  /* LOG_D(PHY,"HARQ_PID %d Rate Matching Segment %d (coded bits %d,E %d, F %d,unpunctured/repeated bits %d, TBS %d, mod_order %d, nb_rb %d, Nl %d, rv %d, round %d)...\n",
        harq_pid,r, G,E,harq_process->F,
        Kr*3,
        harq_process->TBS,
        Qm,
        harq_process->nb_rb,
        harq_process->Nl,
        harq_process->rvidx,
        harq_process->round); */
  //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_RATE_MATCHING, VCD_FUNCTION_IN);

  const int r = rdata->segment_r;
  const uint32_t Tbslbrm = rdata->Tbslbrm;
  if (nr_rate_matching_ldpc_rx(Tbslbrm,
                               p_decoderParms->BG,
                               p_decoderParms->Z,
                               harq_process->d[r],
                               w,
                               harq_process->C,
                               dlsch->dlsch_config.rv,
                               harq_process->first_rx,
                               E,
                               harq_process->F,
                               Kr-harq_process->F-2*(p_decoderParms->Z))==-1) {
    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_RATE_MATCHING, VCD_FUNCTION_OUT);
    stop_meas(&rdata->ts_rate_unmatch);
    LOG_E(PHY,"dlsch_decoding.c: Problem in rate_matching\n");
    rdata->decodeIterations = dlsch->max_ldpc_iterations + 1;
    return;
  }
  stop_meas(&rdata->ts_rate_unmatch);

  if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD)) {
    LOG_D(PHY,"decoder input(segment %u) :",r);

    for (int i=0; i<E; i++)
      LOG_D(PHY,"%d : %d\n",i,harq_process->d[r][i]);

    LOG_D(PHY,"\n");
  }

  int length_dec;
  int crc_type;
  const int A = rdata->A;
  if (harq_process->C == 1) {
    if (A > NR_MAX_PDSCH_TBS)
      crc_type = CRC24_A;
    else
      crc_type = CRC16;

    length_dec = harq_process->B;
  } else {
    crc_type = CRC24_B;
    length_dec = (harq_process->B+24*harq_process->C)/harq_process->C;
  }

  {
    const int kc = rdata->Kc;
    const int K_bits_F = Kr-harq_process->F;
    __attribute__ ((aligned(16))) int16_t  z [68*384 + 16] = {0};
    __attribute__ ((aligned(16))) int8_t   l [68*384 + 16] = {0};

    __m128i *pv = (__m128i*)&z;
    __m128i *pl = (__m128i*)&l;
    start_meas(&rdata->ts_ldpc_decode);
    //set Filler bits
    memset((&z[0]+K_bits_F),127,harq_process->F*sizeof(int16_t));
    //Move coded bits before filler bits
    memcpy((&z[0]+2*harq_process->Z),harq_process->d[r],(K_bits_F-2*harq_process->Z)*sizeof(int16_t));
    //skip filler bits
    memcpy((&z[0]+Kr),harq_process->d[r]+(Kr-2*harq_process->Z),(kc*harq_process->Z-Kr)*sizeof(int16_t));

    //Saturate coded bits before decoding into 8 bits values
    for (int i=0, j=0; j < ((kc*harq_process->Z)>>4)+1;  i+=2, j++) {
      pl[j] = _mm_packs_epi16(pv[i],pv[i+1]);
    }

    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_LDPC, VCD_FUNCTION_IN);
    p_decoderParms->block_length=length_dec;
    int8_t llrProcBuf[OAI_UL_LDPC_MAX_NUM_LLR] __attribute__((aligned(32)));
    nrLDPC_initcall(p_decoderParms, (int8_t*)&pl[0], llrProcBuf);
    t_nrLDPC_time_stats procTime = {0};
    int no_iteration_ldpc = nrLDPC_decoder(p_decoderParms,
                                           (int8_t *)&pl[0],
                                           llrProcBuf,
                                           &procTime);
    //VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_LDPC, VCD_FUNCTION_OUT);

    LOG_D(PHY,"no_iteration_ldpc = %d\n", no_iteration_ldpc);
    // Fixme: correct type is unsigned, but nrLDPC_decoder and all called behind use signed int
    if (check_crc((uint8_t *)llrProcBuf,length_dec,harq_process->F,crc_type)) {
      LOG_D(PHY,"Segment %u CRC OK\n",r);
      if (no_iteration_ldpc > dlsch->max_ldpc_iterations)
        no_iteration_ldpc = dlsch->max_ldpc_iterations;
    } else {
      LOG_D(PHY,"%d.%d CRC NOT OK\n",rdata->proc->frame_rx,rdata->proc->nr_slot_rx);
      no_iteration_ldpc = dlsch->max_ldpc_iterations + 1;
    }

    if (r==0) {
      for (int i=0; i<10; i++) LOG_D(PHY,"byte %d : %x\n",i,((uint8_t *)llrProcBuf)[i]);
    }

    rdata->decodeIterations = no_iteration_ldpc;

    memcpy(harq_process->c[r], llrProcBuf, Kr>>3);

    stop_meas(&rdata->ts_ldpc_decode);
  }
}

uint32_t nr_dlsch_decoding(PHY_VARS_NR_UE *phy_vars_ue,
                           UE_nr_rxtx_proc_t *proc,
                           int eNB_id,
                           short *dlsch_llr,
                           NR_DL_FRAME_PARMS *frame_parms,
                           NR_UE_DLSCH_t *dlsch,
                           NR_DL_UE_HARQ_t *harq_process,
                           uint32_t frame,
                           uint16_t nb_symb_sch,
                           uint8_t nr_slot_rx,
                           uint8_t harq_pid,
                           int b_size,
                           uint8_t b[b_size]) {
  DevAssert(harq_process != NULL);
  DevAssert(dlsch_llr != NULL);

  // HARQ stats
  phy_vars_ue->dl_stats[harq_process->DLround]++;
  LOG_D(PHY,"Round %d RV idx %d\n",harq_process->DLround,dlsch->dlsch_config.rv);
  const int dmrs_Type = dlsch->dlsch_config.dmrsConfigType;
  AssertFatal(dmrs_Type == 0 || dmrs_Type == 1, "Illegal dmrs_type %d\n", dmrs_Type);
  const int nb_re_dmrs = (dmrs_Type==NFAPI_NR_DMRS_TYPE1) ?
                              6*dlsch->dlsch_config.n_dmrs_cdm_groups :
                              4*dlsch->dlsch_config.n_dmrs_cdm_groups;

  const int dmrs_length = get_num_dmrs(dlsch->dlsch_config.dlDmrsSymbPos);
  vcd_signal_dumper_dump_function_by_name(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_SEGMENTATION, VCD_FUNCTION_IN);

  const int nb_rb = dlsch->dlsch_config.number_rbs;
  const int A = dlsch->dlsch_config.TBS;
  dlsch->last_iteration_cnt = dlsch->max_ldpc_iterations + 1;
  harq_process->G = nr_get_G(nb_rb, nb_symb_sch, nb_re_dmrs, dmrs_length, dlsch->dlsch_config.qamModOrder,dlsch->Nl);

  // target_code_rate is in 0.1 units
  const float Coderate = (float) dlsch->dlsch_config.targetCodeRate / 10240.0f;

  LOG_D(PHY,"%d.%d DLSCH Decoding, harq_pid %d TBS %d (%d) G %d nb_re_dmrs %d length dmrs %d mcs %d Nl %d nb_symb_sch %d nb_rb %d Qm %d Coderate %f\n",
        frame,nr_slot_rx,harq_pid,A,A/8,harq_process->G, nb_re_dmrs, dmrs_length, dlsch->dlsch_config.mcs, dlsch->Nl, nb_symb_sch, nb_rb, dlsch->dlsch_config.qamModOrder, Coderate);

  t_nrLDPC_dec_params decParams;

  int kc;
  if ((A <=292) || ((A <= NR_MAX_PDSCH_TBS) && (Coderate <= 0.6667)) || Coderate <= 0.25) {
    decParams.BG = 2;
    kc = 52;
  } else {
    decParams.BG = 1;
    kc = 68;
  }

  if (harq_process->first_rx == 1) {
    // This is a new packet, so compute quantities regarding segmentation
    if (A > NR_MAX_PDSCH_TBS)
      harq_process->B = A+24;
    else
      harq_process->B = A+16;

    nr_segmentation(NULL,
                    NULL,
                    harq_process->B,
                    &harq_process->C,
                    &harq_process->K,
                    &harq_process->Z, // [hna] Z is Zc
                    &harq_process->F,
                    decParams.BG);

    if (harq_process->C>MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*dlsch->Nl) {
      LOG_E(PHY,"nr_segmentation.c: too many segments %d, B %d\n",harq_process->C,harq_process->B);
      return(-1);
    }

    if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD) && (!frame%100))
      LOG_I(PHY,"K %d C %d Z %d nl %d \n", harq_process->K, harq_process->C, harq_process->Z, dlsch->Nl);
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_SEGMENTATION, VCD_FUNCTION_OUT);
  decParams.Z = harq_process->Z;
  decParams.numMaxIter = dlsch->max_ldpc_iterations;
  decParams.outMode= 0;
  uint16_t a_segments = MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*dlsch->Nl;  //number of segments to be allocated

  if (nb_rb != 273) {
    a_segments = a_segments*nb_rb;
    a_segments = a_segments/273 +1;
  }

  DevAssert(harq_process->C <= a_segments);

  if (LOG_DEBUGFLAG(DEBUG_DLSCH_DECOD))
    LOG_I(PHY,"Segmentation: C %d, K %d\n",harq_process->C,harq_process->K);

  const int Kr_bytes = harq_process->K >> 3;
  int offset = 0;
  int r_offset = 0;
  int nbDecode = 0;
  void (*nr_processDLSegment_ptr)(void*) = &nr_processDLSegment;
  notifiedFIFO_t nf;
  initNotifiedFIFO(&nf);
  start_meas(&phy_vars_ue->dlsch_ldpc_whole);
  for (int r=0; r<harq_process->C; r++) {
    //printf("start rx segment %d\n",r);
    const int E = nr_get_E(harq_process->G, harq_process->C, dlsch->dlsch_config.qamModOrder, dlsch->Nl, r);
    decParams.R = nr_get_R_ldpc_decoder(dlsch->dlsch_config.rv, E, decParams.BG, decParams.Z, &harq_process->llrLen, harq_process->DLround);
    union ldpcReqUnion id = {.s={dlsch->rnti,frame,nr_slot_rx,0,0}};
    notifiedFIFO_elt_t *req=newNotifiedFIFO_elt(sizeof(ldpcDecode_ue_t), id.p, &nf, nr_processDLSegment_ptr);
    ldpcDecode_ue_t * rdata=(ldpcDecode_ue_t *) NotifiedFifoData(req);

    rdata->phy_vars_ue = phy_vars_ue;
    rdata->harq_process = harq_process;
    rdata->decoderParms = decParams;
    rdata->dlsch_llr = dlsch_llr;
    rdata->Kc = kc;
    rdata->harq_pid = harq_pid;
    rdata->segment_r = r;
    rdata->nbSegments = harq_process->C;
    rdata->E = E;
    rdata->A = A;
    rdata->Qm = dlsch->dlsch_config.qamModOrder;
    rdata->r_offset = r_offset;
    rdata->Kr_bytes = Kr_bytes;
    rdata->rv_index = dlsch->dlsch_config.rv;
    rdata->Tbslbrm = dlsch->dlsch_config.tbslbrm;
    rdata->offset = offset;
    rdata->dlsch = dlsch;
    rdata->dlsch_id = 0;
    rdata->proc = proc;
    reset_meas(&rdata->ts_deinterleave);
    reset_meas(&rdata->ts_rate_unmatch);
    reset_meas(&rdata->ts_ldpc_decode);
    pushTpool(&get_nrUE_params()->Tpool,req);
    nbDecode++;
    LOG_D(PHY,"Added a block to decode, in pipe: %d\n",nbDecode);
    r_offset += E;
    offset += (Kr_bytes - (harq_process->F>>3) - ((harq_process->C>1)?3:0));
    //////////////////////////////////////////////////////////////////////////////////////////
  }
  int num_seg_ok = 0;
  for (int r=0; r<nbDecode; r++) {
    notifiedFIFO_elt_t *req=pullTpool(&nf,  &get_nrUE_params()->Tpool);
    if (req == NULL)
      break; // Tpool has been stopped
    bool last = false;
    if (r == nbDecode - 1)
      last = true;
    bool stop = nr_ue_postDecode(phy_vars_ue, req, last, &nf, b_size, b, &num_seg_ok, proc);
    delNotifiedFIFO_elt(req);
    if (stop)
      break;
  }
  stop_meas(&phy_vars_ue->dlsch_ldpc_whole);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_DLSCH_COMBINE_SEG, VCD_FUNCTION_OUT);

  return dlsch->last_iteration_cnt;
}
