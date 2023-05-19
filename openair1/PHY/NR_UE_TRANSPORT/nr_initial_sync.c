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

/*! \file PHY/LTE_TRANSPORT/initial_sync.c
* \brief Routines for initial UE synchronization procedure (PSS,SSS,PBCH and frame format detection)
* \author R. Knopp, F. Kaltenberger
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr,kaltenberger@eurecom.fr
* \note
* \warning
*/
#include "PHY/types.h"
#include "PHY/defs_nr_UE.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "nr_transport_proto_ue.h"
#include "PHY/NR_UE_ESTIMATION/nr_estimation.h"
#include "SCHED_NR_UE/defs.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "common/utils/nr/nr_common.h"

#include "common_lib.h"
#include <math.h>

#include "PHY/NR_REFSIG/pss_nr.h"
#include "PHY/NR_REFSIG/sss_nr.h"
#include "PHY/NR_REFSIG/refsig_defs_ue.h"
#include "openair1/PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"

extern openair0_config_t openair0_cfg[];
//static  nfapi_nr_config_request_t config_t;
//static  nfapi_nr_config_request_t* config =&config_t;
int cnt=0;

#define DEBUG_INITIAL_SYNCH
#define DUMP_PBCH_CH_ESTIMATES 0

// create a new node of SSB structure
NR_UE_SSB* create_ssb_node(uint8_t  i, uint8_t  h) {

  NR_UE_SSB *new_node = (NR_UE_SSB*)malloc(sizeof(NR_UE_SSB));
  new_node->i_ssb = i;
  new_node->n_hf = h;
  new_node->c_re = 0;
  new_node->c_im = 0;
  new_node->metric = 0;
  new_node->next_ssb = NULL;

  return new_node;
}


// insertion of the structure in the ordered list (highest metric first)
NR_UE_SSB* insert_into_list(NR_UE_SSB *head, NR_UE_SSB *node) {

  if (node->metric > head->metric) {
    node->next_ssb = head;
    head = node;
    return head;
  }

  NR_UE_SSB *current = head;
  while (current->next_ssb !=NULL) {
    NR_UE_SSB *temp=current->next_ssb;
    if(node->metric > temp->metric) {
      node->next_ssb = temp;
      current->next_ssb = node;
      return head;
    }
    else
      current = temp;
  }
  current->next_ssb = node;

  return head;
}


void free_list(NR_UE_SSB *node) {
  if (node->next_ssb != NULL)
    free_list(node->next_ssb);
  free(node);
}


int nr_pbch_detection(UE_nr_rxtx_proc_t *proc,
                      PHY_VARS_NR_UE *ue,
                      const int pbch_initial_symbol,
                      nr_phy_data_t *phy_data,
                      const c16_t rxdataF[NR_N_SYMBOLS_SSB][ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size])
{
  NR_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  int ret =-1;

  NR_UE_SSB *best_ssb = NULL;
  NR_UE_SSB *current_ssb;

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync: starting PBCH detection (rx_offset %d)\n",ue->Mod_id,
        ue->rx_offset);
#endif

  const uint8_t  N_L = (frame_parms->Lmax == 4)? 4:8;
  const uint8_t  N_hf = (frame_parms->Lmax == 4)? 2:1;

  // loops over possible pbch dmrs cases to retrive best estimated i_ssb (and n_hf for Lmax=4) for multiple ssb detection
  for (int hf = 0; hf < N_hf; hf++) {
    for (int l = 0; l < N_L ; l++) {

      // initialization of structure
      current_ssb = create_ssb_node(l,hf);

      // computing correlation between received DMRS symbols and transmitted sequence for current i_ssb and n_hf
      for(int i=pbch_initial_symbol; i<pbch_initial_symbol+3;i++)
          nr_pbch_dmrs_correlation(ue,proc,i,i-pbch_initial_symbol,current_ssb,rxdataF[i]);
      
      current_ssb->metric = current_ssb->c_re*current_ssb->c_re + current_ssb->c_im*current_ssb->c_im;
      
      // generate a list of SSB structures
      if (best_ssb == NULL)
        best_ssb = current_ssb;
      else
        best_ssb = insert_into_list(best_ssb,current_ssb);

    }
  }

  NR_UE_SSB *temp_ptr=best_ssb;
  while (ret!=0 && temp_ptr != NULL) {

  // computing channel estimation for selected best ssb
    const int estimateSz = frame_parms->symbols_per_slot * frame_parms->ofdm_symbol_size;
    __attribute__ ((aligned(32))) struct complex16 dl_ch_estimates[frame_parms->nb_antennas_rx][estimateSz];
    int16_t pbch_e_rx[NR_POLAR_PBCH_E];

    for(int i=pbch_initial_symbol; i<pbch_initial_symbol+3;i++) {
      for (int aarx = 0; aarx < ue->frame_parms.nb_antennas_rx; aarx++) {
        nr_pbch_channel_estimation(ue,
                                  i-pbch_initial_symbol,
                                  temp_ptr->i_ssb,
                                  temp_ptr->n_hf,
                                  rxdataF[i][aarx],
                                  dl_ch_estimates);
      }
      nr_generate_pbch_llr(ue, i-pbch_initial_symbol, temp_ptr->i_ssb, rxdataF[i], dl_ch_estimates, pbch_e_rx);
    }
    fapiPbch_t pbchResult = {0}; /* TODO: Not used anywhere. To be cleaned later */
    ret = nr_pbch_decode(ue, proc, temp_ptr->i_ssb, pbch_e_rx, &pbchResult, phy_data);

    if (DUMP_PBCH_CH_ESTIMATES && (ret == 0)) {
      write_output("pbch_ch_estimates.m", "pbch_ch_estimates", dl_ch_estimates, frame_parms->nb_antennas_rx*estimateSz, 1, 1);
    }

    temp_ptr=temp_ptr->next_ssb;
  }

  free_list(best_ssb);

  
  if (ret==0) {
    
    frame_parms->nb_antenna_ports_gNB = 1; //pbch_tx_ant;
    
    // set initial transmission mode to 1 or 2 depending on number of detected TX antennas
    //frame_parms->mode1_flag = (pbch_tx_ant==1);
    // openair_daq_vars.dlsch_transmission_mode = (pbch_tx_ant>1) ? 2 : 1;


#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"[UE%d] Initial sync: pbch decoded sucessfully\n",ue->Mod_id);
#endif
    return(0);
  } else {
    return(-1);
  }

}

char duplex_string[2][4] = {"FDD","TDD"};
char prefix_string[2][9] = {"NORMAL","EXTENDED"};

int nr_initial_sync(UE_nr_rxtx_proc_t *proc,
                    PHY_VARS_NR_UE *ue,
                    int n_frames, int sa)
{

  int32_t sync_pos, sync_pos_frame; // k_ssb, N_ssb_crb, sync_pos2,
  int32_t metric_tdd_ncp=0;
  uint8_t phase_tdd_ncp;
  double im, re;
  int is;

  NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  int ret=-1;
  int rx_power=0; //aarx,

  nr_phy_data_t phy_data = {0};
  NR_UE_PDCCH_CONFIG *phy_pdcch_config = &phy_data.phy_pdcch_config;
  
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_NR_INITIAL_UE_SYNC, VCD_FUNCTION_IN);


  LOG_D(PHY,"nr_initial sync ue RB_DL %d\n", fp->N_RB_DL);

  /*   Initial synchronisation
   *
  *                                 1 radio frame = 10 ms
  *     <--------------------------------------------------------------------------->
  *     -----------------------------------------------------------------------------
  *     |                                 Received UE data buffer                    |
  *     ----------------------------------------------------------------------------
  *                     --------------------------
  *     <-------------->| pss | pbch | sss | pbch |
  *                     --------------------------
  *          sync_pos            SS/PBCH block
  */

  cnt++;
  if (1){ // (cnt>100)
   cnt =0;

   // initial sync performed on two successive frames, if pbch passes on first frame, no need to process second frame 
   // only one frame is used for symulation tools
   for(is=0; is<n_frames;is++) {

    /* process pss search on received buffer */
    sync_pos = pss_synchro_nr(ue, is, NO_RATE_CHANGE);
    if (sync_pos < fp->nb_prefix_samples)
      continue;

    ue->ssb_offset = sync_pos - fp->nb_prefix_samples;

#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n", ue->Mod_id, sync_pos,ue->common_vars.eNb_id);
    LOG_I(PHY,"sync_pos %d ssb_offset %d \n",sync_pos,ue->ssb_offset);
#endif

    /* check that SSS/PBCH block is continuous inside the received buffer */
    if (ue->ssb_offset + NR_N_SYMBOLS_SSB * (fp->ofdm_symbol_size + fp->nb_prefix_samples) < fp->samples_per_frame) {

      // digital compensation of FFO for SSB symbols
      if (ue->UE_fo_compensation){
        double s_time = 1/(1.0e3*fp->samples_per_subframe);  // sampling time
        double off_angle = -2*M_PI*s_time*(ue->common_vars.freq_offset);  // offset rotation angle compensation per sample

        // In SA we need to perform frequency offset correction until the end of buffer because we need to decode SIB1
        // and we do not know yet in which slot it goes.

        // start for offset correction
        int start = is*fp->samples_per_frame;

        // loop over samples
        int end = start + fp->samples_per_frame;

        for(int n=start; n<end; n++){
          for (int ar=0; ar<fp->nb_antennas_rx; ar++) {
            re = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n]);
            im = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n+1]);
            ((short *)ue->common_vars.rxdata[ar])[2*n] = (short)(round(re*cos(n*off_angle) - im*sin(n*off_angle)));
            ((short *)ue->common_vars.rxdata[ar])[2*n+1] = (short)(round(re*sin(n*off_angle) + im*cos(n*off_angle)));
          }
        }
      }

      /* slop_fep function works for lte and takes into account begining of frame with prefix for subframe 0 */
      /* for NR this is not the case but slot_fep is still used for computing FFT of samples */
      /* in order to achieve correct processing for NR prefix samples is forced to 0 and then restored after function call */
      /* symbol number are from beginning of SS/PBCH blocks as below:  */
      /*    Signal            PSS  PBCH  SSS  PBCH                     */
      /*    symbol number      0     1    2    3                       */
      /* time samples in buffer rxdata are used as input of FFT -> FFT results are stored in the frequency buffer rxdataF */
      /* rxdataF stores SS/PBCH from beginning of buffers in the same symbol order as in time domain */

      __attribute__ ((aligned(32))) c16_t rxdata[NR_N_SYMBOLS_SSB][fp->nb_antennas_rx][fp->ofdm_symbol_size + fp->nb_prefix_samples0];
      __attribute__ ((aligned(32))) c16_t rxdataF[NR_N_SYMBOLS_SSB][fp->nb_antennas_rx][fp->ofdm_symbol_size];

      for (int aarx = 0; aarx < fp->nb_antennas_rx; aarx++) {
        const int symbolSamples = (fp->ofdm_symbol_size + fp->nb_prefix_samples);
        for (int symbol = 0; symbol < NR_N_SYMBOLS_SSB; symbol++) {
          memcpy(rxdata[symbol][aarx],
                 &ue->common_vars.rxdata[aarx][is*fp->samples_per_frame + ue->ssb_offset + (symbol*symbolSamples) + fp->nb_prefix_samples],
                 sizeof(c16_t)*fp->ofdm_symbol_size);
        }
      }

      for (int i = 0; i < NR_N_SYMBOLS_SSB; i++) {
        nr_symbol_fep(ue, proc->nr_slot_rx, i, rxdata[i], rxdataF[i]);
      }

#ifdef DEBUG_INITIAL_SYNCH
      LOG_I(PHY,"Calling sss detection (normal CP)\n");
#endif

      int freq_offset_sss = 0;
      ret = rx_sss_nr(ue, proc, &metric_tdd_ncp, &phase_tdd_ncp, &freq_offset_sss, rxdataF);

      // digital compensation of FFO for SSB symbols
      if (ue->UE_fo_compensation){
        double s_time = 1/(1.0e3*fp->samples_per_subframe);  // sampling time
        double off_angle = -2*M_PI*s_time*freq_offset_sss;   // offset rotation angle compensation per sample

        // In SA we need to perform frequency offset correction until the end of buffer because we need to decode SIB1
        // and we do not know yet in which slot it goes.

        // start for offset correction
        int start = is*fp->samples_per_frame;

        // loop over samples
        int end = start + fp->samples_per_frame;

        for(int n=start; n<end; n++){
          for (int ar=0; ar<fp->nb_antennas_rx; ar++) {
            re = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n]);
            im = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n+1]);
            ((short *)ue->common_vars.rxdata[ar])[2*n] = (short)(round(re*cos(n*off_angle) - im*sin(n*off_angle)));
            ((short *)ue->common_vars.rxdata[ar])[2*n+1] = (short)(round(re*sin(n*off_angle) + im*cos(n*off_angle)));
          }
        }

        ue->common_vars.freq_offset += freq_offset_sss;
      }

      if (ret==0) { //we got sss channel
        nr_gold_pbch(ue);
        ret = nr_pbch_detection(proc, ue, 1, &phy_data, rxdataF);  // start pbch detection at first symbol after pss
      }

      if (ret == 0) {

        // sync at symbol ue->symbol_offset
        // computing the offset wrt the beginning of the frame
        int mu = fp->numerology_index;
        // number of symbols with different prefix length
        // every 7*(1<<mu) symbols there is a different prefix length (38.211 5.3.1)
        int n_symb_prefix0 = (ue->symbol_offset/(7*(1<<mu)))+1;
        sync_pos_frame = n_symb_prefix0*(fp->ofdm_symbol_size + fp->nb_prefix_samples0)+(ue->symbol_offset-n_symb_prefix0)*(fp->ofdm_symbol_size + fp->nb_prefix_samples);
        // for a correct computation of frame number to sync with the one decoded at MIB we need to take into account in which of the n_frames we got sync
        ue->init_sync_frame = n_frames - 1 - is;

        // compute the scramblingID_pdcch and the gold pdcch
        ue->scramblingID_pdcch = fp->Nid_cell;
        nr_gold_pdcch(ue,fp->Nid_cell);

        // compute the scrambling IDs for PDSCH DMRS
        for (int i=0; i<NR_NB_NSCID; i++) {
          ue->scramblingID_dlsch[i]=fp->Nid_cell;
          nr_gold_pdsch(i, ue->scramblingID_dlsch[i], ue);
        }

        nr_init_csi_rs(fp, ue->nr_csi_info->nr_gold_csi_rs, fp->Nid_cell);

        // initialize the pusch dmrs
        for (int i=0; i<NR_NB_NSCID; i++) {
          ue->scramblingID_ulsch[i]=fp->Nid_cell;
          nr_init_pusch_dmrs(ue, ue->scramblingID_ulsch[i], i);
        }


        // we also need to take into account the shift by samples_per_frame in case the if is true
        if (ue->ssb_offset < sync_pos_frame){
          ue->rx_offset = fp->samples_per_frame - sync_pos_frame + ue->ssb_offset;
          ue->init_sync_frame += 1;
        }
        else
          ue->rx_offset = ue->ssb_offset - sync_pos_frame;
      }   

    /*
    int nb_prefix_samples0 = fp->nb_prefix_samples0;
    fp->nb_prefix_samples0 = fp->nb_prefix_samples;
	  
    nr_slot_fep(ue, proc, 0, 0, ue->ssb_offset, 0, NR_PDCCH_EST);
    nr_slot_fep(ue, proc, 1, 0, ue->ssb_offset, 0, NR_PDCCH_EST);
    fp->nb_prefix_samples0 = nb_prefix_samples0;	

    LOG_I(PHY,"[UE  %d] AUTOTEST Cell Sync : frame = %d, rx_offset %d, freq_offset %d \n",
              ue->Mod_id,
              ue->proc.proc_rxtx[0].frame_rx,
              ue->rx_offset,
              ue->common_vars.freq_offset );
    */


#ifdef DEBUG_INITIAL_SYNCH
      LOG_I(PHY,"TDD Normal prefix: CellId %d metric %d, phase %d, pbch %d\n",
            fp->Nid_cell,metric_tdd_ncp,phase_tdd_ncp,ret);
#endif

      }
      else {
#ifdef DEBUG_INITIAL_SYNCH
       LOG_I(PHY,"TDD Normal prefix: SSS error condition: sync_pos %d\n", sync_pos);
#endif
      }
      if (ret == 0) break;
    }
  }
  else {
    ret = -1;
  }

  /* Consider this is a false detection if the offset is > 1000 Hz 
     Not to be used now that offest estimation is in place
  if( (abs(ue->common_vars.freq_offset) > 150) && (ret == 0) )
  {
	  ret=-1;
	  LOG_E(HW, "Ignore MIB with high freq offset [%d Hz] estimation \n",ue->common_vars.freq_offset);
  }*/

  if (ret==0) {  // PBCH found so indicate sync to higher layers and configure frame parameters

    //#ifdef DEBUG_INITIAL_SYNCH

    LOG_I(PHY, "[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);

    //#endif

    if (ue->UE_scan_carrier == 0) {

    #if UE_AUTOTEST_TRACE
      LOG_I(PHY,"[UE  %d] AUTOTEST Cell Sync : rx_offset %d, freq_offset %d \n",
              ue->Mod_id,
              ue->rx_offset,
              ue->common_vars.freq_offset );
    #endif

// send sync status to higher layers later when timing offset converge to target timing

    }


#if defined(OAI_USRP) || defined(OAI_BLADERF) || defined(OAI_LMSSDR) || defined(OAI_ADRV9371_ZC706)
    LOG_I(PHY, "[UE %d] Measured Carrier Frequency %.0f Hz (offset %d Hz)\n",
	  ue->Mod_id,
	  openair0_cfg[0].rx_freq[0]+ue->common_vars.freq_offset,
	  ue->common_vars.freq_offset);
#endif
  } else {
#ifdef DEBUG_INITIAL_SYNC
    LOG_I(PHY,"[UE%d] Initial sync : PBCH not ok\n",ue->Mod_id);
    LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,sync_pos,ue->common_vars.eNb_id);
    LOG_I(PHY,"[UE%d] Initial sync : Estimated Nid_cell %d, Frame_type %d\n",ue->Mod_id,
          frame_parms->Nid_cell,frame_parms->frame_type);
#endif

  }

  // gain control
  if (ret!=0) { //we are not synched, so we cannot use rssi measurement (which is based on channel estimates)
    rx_power = 0;

    // do a measurement on the best guess of the PSS
    //for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
    //  rx_power += signal_energy(&ue->common_vars.rxdata[aarx][sync_pos2],
	//			frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples);

    /*
    // do a measurement on the full frame
    for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
      rx_power += signal_energy(&ue->common_vars.rxdata[aarx][0],
				frame_parms->samples_per_subframe*10);
    */

    // we might add a low-pass filter here later
    ue->measurements.rx_power_avg[0] = rx_power/fp->nb_antennas_rx;

    ue->measurements.rx_power_avg_dB[0] = dB_fixed(ue->measurements.rx_power_avg[0]);

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync : Estimated power: %d dB\n",ue->Mod_id,ue->measurements.rx_power_avg_dB[0] );
#endif

#ifndef OAI_USRP
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
#ifndef OAI_ADRV9371_ZC706
  //phy_adjust_gain(ue,ue->measurements.rx_power_avg_dB[0],0);
#endif
#endif
#endif
#endif

  }
  else {

#ifndef OAI_USRP
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
#ifndef OAI_ADRV9371_ZC706
  //phy_adjust_gain(ue,dB_fixed(ue->measurements.rssi),0);
#endif
#endif
#endif
#endif

  }

  if (ue->target_Nid_cell != -1) {
    return ret;
  }

  // if stand alone and sync on ssb do sib1 detection as part of initial sync
  if (sa == 1 && ret == 0) {
    __attribute__ ((aligned(32))) c16_t rxdata[NR_SYMBOLS_PER_SLOT][fp->nb_antennas_rx][fp->ofdm_symbol_size + fp->nb_prefix_samples0];

    for (int aarx = 0; aarx < fp->nb_antennas_rx; aarx++) {
      int sampleOffset = 0;
      for (int symbol = 0; symbol < NR_SYMBOLS_PER_SLOT; symbol++) {
        const int samplesPrefix = (symbol % (0x7 << fp->numerology_index)) ? fp->nb_prefix_samples : fp->nb_prefix_samples0;
        memcpy(rxdata[symbol][aarx],
               &ue->common_vars.rxdata[aarx][is*fp->samples_per_frame + ue->rx_offset + sampleOffset + samplesPrefix],
               sizeof(c16_t)*fp->ofdm_symbol_size);
        sampleOffset += (samplesPrefix + fp->ofdm_symbol_size);
      }
    }

    bool dec = false;
    proc->gNB_id = 0; //FIXME

    int16_t *pdcchLlr = NULL;
    c16_t (*pdsch_ch_estiamtes)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size] = NULL;
    c16_t (*pdsch_dl_ch_est_ext)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*rxdataF_ext)[NR_SYMBOLS_PER_SLOT][ue->frame_parms.nb_antennas_rx]
      [phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*rxdataF_comp)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*dl_ch_mag)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*dl_ch_magb)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*dl_ch_magr)[NR_SYMBOLS_PER_SLOT][phy_data.dlsch[0].Nl]
      [ue->frame_parms.nb_antennas_rx][phy_data.dlsch[0].dlsch_config.number_rbs] = NULL;
    c16_t (*ptrs_phase)[NR_SYMBOLS_PER_SLOT][ue->frame_parms.nb_antennas_rx] = NULL;
    int32_t (*ptrs_re)[NR_SYMBOLS_PER_SLOT][ue->frame_parms.nb_antennas_rx] = NULL;
    nr_pdcch_slot_init(&phy_data, ue);
    nr_pdsch_slot_init(&phy_data, ue);

    for (int symbol = 0; symbol < NR_SYMBOLS_PER_SLOT; symbol++) {
      __attribute__ ((aligned(32))) c16_t rxdataF[fp->nb_antennas_rx][fp->ofdm_symbol_size];
      nr_symbol_fep(ue, phy_pdcch_config->slot, symbol, rxdata[symbol], rxdataF);

      /* Process PDCCH */
      const NR_UE_PDCCH_CONFIG *phy_pdcch_config = &phy_data.phy_pdcch_config;
      const int nb_symb_pdcch = get_max_pdcch_symb(phy_pdcch_config);
      const int start_symb_pdcch = get_min_pdcch_start_symb(phy_pdcch_config);
      const int last_symb_pdcch = start_symb_pdcch + nb_symb_pdcch - 1;
      const int pdcchLlrSize = get_pdcch_max_rbs(phy_pdcch_config) * nb_symb_pdcch * 9 * 2;
      if (!pdcchLlr) pdcchLlr = malloc16_clear(sizeof(*pdcchLlr) * pdcchLlrSize * phy_pdcch_config->nb_search_space);
      nr_pdcch_generate_llr(ue, proc, symbol, &phy_data, pdcchLlrSize, rxdataF, pdcchLlr);
      if (last_symb_pdcch == symbol) {
        nr_pdcch_dci_indication(proc, pdcchLlrSize, ue, &phy_data, pdcchLlr);
        free(pdcchLlr); pdcchLlr = NULL;
      }

      const int first_pdsch_symbol = phy_data.dlsch[0].dlsch_config.start_symbol;
      const int last_pdsch_symbol = phy_data.dlsch[0].dlsch_config.start_symbol +
                                    phy_data.dlsch[0].dlsch_config.number_symbols - 1;
      if (!pdsch_ch_estiamtes) pdsch_ch_estiamtes = malloc16_clear(sizeof(*pdsch_ch_estiamtes));
      if (!rxdataF_ext) rxdataF_ext = malloc16_clear(sizeof(*rxdataF_ext));
      if (!pdsch_dl_ch_est_ext) pdsch_dl_ch_est_ext = malloc16_clear(sizeof(*pdsch_dl_ch_est_ext));
      if (!rxdataF_comp) rxdataF_comp = malloc16_clear(sizeof(*rxdataF_comp));
      if (!dl_ch_mag) dl_ch_mag = malloc16_clear(sizeof(*dl_ch_mag));
      if (!dl_ch_magb) dl_ch_magb = malloc16_clear(sizeof(*dl_ch_magb));
      if (!dl_ch_magr) dl_ch_magr = malloc16_clear(sizeof(*dl_ch_magr));
      if (!ptrs_phase) ptrs_phase = malloc16_clear(sizeof(*ptrs_phase));
      if (!ptrs_re) ptrs_re = malloc16_clear(sizeof(*ptrs_re));
      if ((symbol < last_pdsch_symbol) &&
          (symbol >= first_pdsch_symbol)) {
        nr_pdsch_generate_channel_estimates(ue, proc, symbol, &phy_data.dlsch[0], rxdataF, (*pdsch_ch_estiamtes)[symbol]);
        nr_generate_pdsch_extracted_rxdataF(ue, proc, symbol, &phy_data.dlsch[0], rxdataF, (*rxdataF_ext)[symbol]);
      } else if (symbol == last_pdsch_symbol) {
        nr_pdsch_generate_channel_estimates(ue, proc, symbol, &phy_data.dlsch[0], rxdataF, (*pdsch_ch_estiamtes)[symbol]);
        nr_generate_pdsch_extracted_rxdataF(ue, proc, symbol, &phy_data.dlsch[0], rxdataF, (*rxdataF_ext)[symbol]);
        nr_ue_symb_data_t param = {.dl_ch_mag = (c16_t *)dl_ch_mag, .dl_ch_magb = (c16_t *)dl_ch_magb, .dl_ch_magr = (c16_t *)dl_ch_magr,
                                  .pdsch_dl_ch_est_ext = (c16_t *)pdsch_dl_ch_est_ext, .pdsch_dl_ch_estimates = (c16_t *)pdsch_ch_estiamtes,
                                  .rxdataF_comp = (c16_t *)rxdataF_comp, .rxdataF_ext = (c16_t *)rxdataF_ext,
                                  .ptrs_phase_per_slot = (c16_t *)ptrs_phase, .ptrs_re_per_slot = (int32_t *)ptrs_re,
                                  .symbol = symbol, .UE = ue, .proc = proc};
        dec = nr_ue_pdsch_procedures((void *)&param);
        free(rxdataF_ext); rxdataF_ext = NULL;
        free(pdsch_dl_ch_est_ext); pdsch_dl_ch_est_ext = NULL;
        free(rxdataF_comp); rxdataF_comp = NULL;
        free(dl_ch_mag); dl_ch_mag = NULL;
        free(dl_ch_magb); dl_ch_magb = NULL;
        free(dl_ch_magr); dl_ch_magr = NULL;
        free(ptrs_phase); ptrs_phase = NULL;
        free(ptrs_re); ptrs_re = NULL;
      }
    }

    if (dec == false) // sib1 not decoded
      ret = -1;
  }
  //  exit_fun("debug exit");
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_NR_INITIAL_UE_SYNC, VCD_FUNCTION_OUT);
  return ret;
}

