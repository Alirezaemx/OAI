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

/*! \file phy_procedures_nr_ue.c
 * \brief Implementation of UE procedures from 36.213 LTE specifications
 * \author R. Knopp, F. Kaltenberger, N. Nikaein, A. Mico Pereperez, G. Casati
 * \date 2018
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr, guido.casati@iis.fraunhofer.de
 * \note
 * \warning
 */

#define _GNU_SOURCE

#include "nr/nr_common.h"
#include "assertions.h"
#include "defs.h"
#include "PHY/defs_nr_UE.h"
#include "PHY/NR_REFSIG/dmrs_nr.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "PHY/INIT/nr_phy_init.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_ue.h"
#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"
#include "PHY/NR_UE_TRANSPORT/srs_modulation_nr.h"
#include "SCHED_NR_UE/phy_sch_processing_time.h"
#include "PHY/NR_UE_ESTIMATION/nr_estimation.h"
#ifdef EMOS
#include "SCHED/phy_procedures_emos.h"
#endif
#include "executables/softmodem-common.h"
#include "executables/nr-uesoftmodem.h"
#include "SCHED_NR_UE/pucch_uci_ue_nr.h"
#include <openair1/PHY/TOOLS/phy_scope_interface.h>
#include "PHY/NR_REFSIG/ptrs_nr.h"
#include "refsig_defs_ue.h"

//#define DEBUG_PHY_PROC
#define NR_PDCCH_SCHED
//#define NR_PDCCH_SCHED_DEBUG
//#define NR_PUCCH_SCHED
//#define NR_PUCCH_SCHED_DEBUG
//#define NR_PDSCH_DEBUG

#ifndef PUCCH
#define PUCCH
#endif

#include "common/utils/LOG/log.h"

#ifdef EMOS
fifo_dump_emos_UE emos_dump_UE;
#endif

#include "common/utils/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"
#include "intertask_interface.h"
#include "T.h"

#if defined(OAI_USRP) || defined(OAI_BLADERF) || defined(OAI_LMSSDR) || defined(OAI_ADRV9371_ZC706)
extern uint64_t downlink_frequency[MAX_NUM_CCs][4];
#endif

unsigned int gain_table[31] = {100,112,126,141,158,178,200,224,251,282,316,359,398,447,501,562,631,708,794,891,1000,1122,1258,1412,1585,1778,1995,2239,2512,2818,3162};

void nr_fill_dl_indication(nr_downlink_indication_t *dl_ind,
                           const fapi_nr_dci_indication_t *dci_ind,
                           const fapi_nr_rx_indication_t *rx_ind,
                           const UE_nr_rxtx_proc_t *proc,
                           const PHY_VARS_NR_UE *ue,
                           const void *phy_data)
{

  memset((void*)dl_ind, 0, sizeof(nr_downlink_indication_t));

  dl_ind->gNB_index = proc->gNB_id;
  dl_ind->module_id = ue->Mod_id;
  dl_ind->cc_id     = ue->CC_id;
  dl_ind->frame     = proc->frame_rx;
  dl_ind->slot      = proc->nr_slot_rx;
  dl_ind->phy_data  = phy_data;

  if (dci_ind) {

    dl_ind->rx_ind = NULL; //no data, only dci for now
    dl_ind->dci_ind = dci_ind;

  } else if (rx_ind) {

    dl_ind->rx_ind = rx_ind; //  hang on rx_ind instance
    dl_ind->dci_ind = NULL;

  }
}

void nr_fill_rx_indication(fapi_nr_rx_indication_t *rx_ind,
                           const uint8_t pdu_type,
                           const PHY_VARS_NR_UE *ue,
                           const NR_UE_DLSCH_t *dlsch0,
                           const NR_UE_DLSCH_t *dlsch1,
                           const uint16_t n_pdus,
                           const UE_nr_rxtx_proc_t *proc,
                           const void *typeSpecific,
                           const uint8_t *b)
{
  if (n_pdus > 1){
    LOG_E(PHY, "In %s: multiple number of DL PDUs not supported yet...\n", __FUNCTION__);
  }

  NR_DL_UE_HARQ_t *dl_harq0 = NULL;

  if ((pdu_type !=  FAPI_NR_RX_PDU_TYPE_SSB) && dlsch0) {
    dl_harq0 = &ue->dl_harq_processes[0][dlsch0->dlsch_config.harq_process_nbr];
    trace_NRpdu(DIRECTION_DOWNLINK,
		b,
		dlsch0->dlsch_config.TBS / 8,
		WS_C_RNTI,
		dlsch0->rnti,
		proc->frame_rx,
		proc->nr_slot_rx,
		0,0);
  }
  switch (pdu_type){
    case FAPI_NR_RX_PDU_TYPE_SIB:
    case FAPI_NR_RX_PDU_TYPE_RAR:
    case FAPI_NR_RX_PDU_TYPE_DLSCH:
      if(dlsch0) {
        dl_harq0 = &ue->dl_harq_processes[0][dlsch0->dlsch_config.harq_process_nbr];
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.harq_pid = dlsch0->dlsch_config.harq_process_nbr;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.ack_nack = dl_harq0->ack;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu = b;
        rx_ind->rx_indication_body[n_pdus - 1].pdsch_pdu.pdu_length = dlsch0->dlsch_config.TBS / 8;
      }
      if(dlsch1) {
        AssertFatal(1==0,"Second codeword currently not supported\n");
      }
      break;
    case FAPI_NR_RX_PDU_TYPE_SSB: {
        fapi_nr_ssb_pdu_t *ssb_pdu = &rx_ind->rx_indication_body[n_pdus - 1].ssb_pdu;
        if(typeSpecific) {
          NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
          fapiPbch_t *pbch = (fapiPbch_t *)typeSpecific;
          memcpy(ssb_pdu->pdu, pbch->decoded_output, sizeof(pbch->decoded_output));
          ssb_pdu->additional_bits = pbch->xtra_byte;
          ssb_pdu->ssb_index = (frame_parms->ssb_index)&0x7;
          ssb_pdu->ssb_length = frame_parms->Lmax;
          ssb_pdu->cell_id = frame_parms->Nid_cell;
          ssb_pdu->ssb_start_subcarrier = frame_parms->ssb_start_subcarrier;
          ssb_pdu->rsrp_dBm = ue->measurements.ssb_rsrp_dBm[frame_parms->ssb_index];
          ssb_pdu->decoded_pdu = true;
        }
        else
          ssb_pdu->decoded_pdu = false;
      }
    break;
    case FAPI_NR_CSIRS_IND:
      memcpy(&rx_ind->rx_indication_body[n_pdus - 1].csirs_measurements,
             (fapi_nr_csirs_measurements_t*)typeSpecific,
             sizeof(*(fapi_nr_csirs_measurements_t*)typeSpecific));
      break;
    default:
    break;
  }

  rx_ind->rx_indication_body[n_pdus -1].pdu_type = pdu_type;
  rx_ind->number_pdus = n_pdus;

}

int get_tx_amp_prach(int power_dBm, int power_max_dBm, int N_RB_UL){

  int gain_dB = power_dBm - power_max_dBm, amp_x_100 = -1;

  switch (N_RB_UL) {
  case 6:
  amp_x_100 = AMP;      // PRACH is 6 PRBS so no scale
  break;
  case 15:
  amp_x_100 = 158*AMP;  // 158 = 100*sqrt(15/6)
  break;
  case 25:
  amp_x_100 = 204*AMP;  // 204 = 100*sqrt(25/6)
  break;
  case 50:
  amp_x_100 = 286*AMP;  // 286 = 100*sqrt(50/6)
  break;
  case 75:
  amp_x_100 = 354*AMP;  // 354 = 100*sqrt(75/6)
  break;
  case 100:
  amp_x_100 = 408*AMP;  // 408 = 100*sqrt(100/6)
  break;
  default:
  LOG_E(PHY, "Unknown PRB size %d\n", N_RB_UL);
  return (amp_x_100);
  break;
  }
  if (gain_dB < -30) {
    return (amp_x_100/3162);
  } else if (gain_dB > 0)
    return (amp_x_100);
  else
    return (amp_x_100/gain_table[-gain_dB]);  // 245 corresponds to the factor sqrt(25/6)

  return (amp_x_100);
}

// UL time alignment procedures:
// - If the current tx frame and slot match the TA configuration
//   then timing advance is processed and set to be applied in the next UL transmission
// - Application of timing adjustment according to TS 38.213 p4.2
// todo:
// - handle RAR TA application as per ch 4.2 TS 38.213
void ue_ta_procedures(PHY_VARS_NR_UE *ue, int slot_tx, int frame_tx)
{
  if (frame_tx == ue->ta_frame && slot_tx == ue->ta_slot) {

    uint16_t ofdm_symbol_size = ue->frame_parms.ofdm_symbol_size;

    // convert time factor "16 * 64 * T_c / (2^mu)" in N_TA calculation in TS38.213 section 4.2 to samples by multiplying with samples per second
    //   16 * 64 * T_c            / (2^mu) * samples_per_second
    // = 16 * T_s                 / (2^mu) * samples_per_second
    // = 16 * 1 / (15 kHz * 2048) / (2^mu) * (15 kHz * 2^mu * ofdm_symbol_size)
    // = 16 * 1 /           2048           *                  ofdm_symbol_size
    // = 16 * ofdm_symbol_size / 2048
    uint16_t bw_scaling = 16 * ofdm_symbol_size / 2048;

    ue->timing_advance += (ue->ta_command - 31) * bw_scaling;

    LOG_D(PHY, "[UE %d] [%d.%d] Got timing advance command %u from MAC, new value is %d\n",
          ue->Mod_id,
          frame_tx,
          slot_tx,
          ue->ta_command,
          ue->timing_advance);

    ue->ta_frame = -1;
    ue->ta_slot = -1;
  }
}

void phy_procedures_nrUE_TX(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            nr_phy_data_tx_t *phy_data) {

  int slot_tx = proc->nr_slot_tx;
  int frame_tx = proc->frame_tx;
  int gNB_id = proc->gNB_id;

  AssertFatal(ue->CC_id == 0, "Transmission on secondary CCs is not supported yet\n");

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX,VCD_FUNCTION_IN);

  for(int i=0; i< ue->frame_parms.nb_antennas_tx; ++i)
    memset(ue->common_vars.txdataF[i], 0, sizeof(int)*14*ue->frame_parms.ofdm_symbol_size);

  LOG_D(PHY,"****** start TX-Chain for AbsSubframe %d.%d ******\n", frame_tx, slot_tx);

  start_meas(&ue->phy_proc_tx);

  for (uint8_t harq_pid = 0; harq_pid < NR_MAX_ULSCH_HARQ_PROCESSES; harq_pid++) {
    if (ue->ul_harq_processes[harq_pid].status == ACTIVE)
      nr_ue_ulsch_procedures(ue, harq_pid, frame_tx, slot_tx, gNB_id, phy_data);
  }

  ue_srs_procedures_nr(ue, proc);

  pucch_procedures_ue_nr(ue, proc, phy_data);

  LOG_D(PHY, "Sending Uplink data \n");
  nr_ue_pusch_common_procedures(ue,
                                proc->nr_slot_tx,
                                &ue->frame_parms,
                                ue->frame_parms.nb_antennas_tx);

  nr_ue_prach_procedures(ue, proc);

  LOG_D(PHY,"****** end TX-Chain for AbsSubframe %d.%d ******\n", proc->frame_tx, proc->nr_slot_tx);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX, VCD_FUNCTION_OUT);
  stop_meas(&ue->phy_proc_tx);
}

void nr_ue_measurement_procedures(uint16_t l,
                                  PHY_VARS_NR_UE *ue,
                                  UE_nr_rxtx_proc_t *proc,
                                  NR_UE_DLSCH_t *dlsch,
                                  uint32_t pdsch_est_size,
                                  int32_t dl_ch_estimates[][pdsch_est_size]) {

  NR_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  int frame_rx   = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  int gNB_id = proc->gNB_id;
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_MEASUREMENT_PROCEDURES, VCD_FUNCTION_IN);

  if (l==2) {

    LOG_D(PHY,"Doing UE measurement procedures in symbol l %u Ncp %d nr_slot_rx %d, rxdata %p\n",
      l,
      ue->frame_parms.Ncp,
      nr_slot_rx,
      ue->common_vars.rxdata);

    nr_ue_measurements(ue, proc, dlsch, pdsch_est_size, dl_ch_estimates);

#if T_TRACER
    if(nr_slot_rx == 0)
      T(T_UE_PHY_MEAS, T_INT(gNB_id),  T_INT(ue->Mod_id), T_INT(frame_rx%1024), T_INT(nr_slot_rx),
	T_INT((int)(10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB)),
	T_INT((int)ue->measurements.rx_rssi_dBm[0]),
	T_INT((int)(ue->measurements.rx_power_avg_dB[0] - ue->measurements.n0_power_avg_dB)),
	T_INT((int)ue->measurements.rx_power_avg_dB[0]),
	T_INT((int)ue->measurements.n0_power_avg_dB),
	T_INT((int)ue->measurements.wideband_cqi_avg[0]),
	T_INT((int)ue->common_vars.freq_offset));
#endif
  }

  // accumulate and filter timing offset estimation every subframe (instead of every frame)
  if (( nr_slot_rx == 2) && (l==(2-frame_parms->Ncp))) {

    // AGC

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GAIN_CONTROL, VCD_FUNCTION_IN);


    //printf("start adjust gain power avg db %d\n", ue->measurements.rx_power_avg_dB[gNB_id]);
    phy_adjust_gain_nr (ue,ue->measurements.rx_power_avg_dB[gNB_id],gNB_id);
    
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GAIN_CONTROL, VCD_FUNCTION_OUT);

}

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_MEASUREMENT_PROCEDURES, VCD_FUNCTION_OUT);
}

static void nr_ue_pbch_procedures(PHY_VARS_NR_UE *ue,
                                  UE_nr_rxtx_proc_t *proc,
                                  const c16_t dl_ch_estimates[][ue->frame_parms.ofdm_symbol_size],
                                  const int16_t pbch_e_rx[NR_POLAR_PBCH_E],
                                  nr_phy_data_t *phy_data)
{
  int ret = 0;
  DevAssert(ue);

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  int gNB_id = proc->gNB_id;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PBCH_PROCEDURES, VCD_FUNCTION_IN);

  LOG_D(PHY,"[UE  %d] Frame %d Slot %d, Trying PBCH (NidCell %d, gNB_id %d)\n",ue->Mod_id,frame_rx,nr_slot_rx,ue->frame_parms.Nid_cell,gNB_id);
  fapiPbch_t result;
  ret = nr_rx_pbch(ue,
                   proc,
                   estimateSz,
                   dl_ch_estimates,
                   &ue->frame_parms,
                   (ue->frame_parms.ssb_index)&7,
                   SISO,
                   phy_data,
                   &result,
                   rxdataF);

  if (ret==0) {

#ifdef DEBUG_PHY_PROC
    uint16_t frame_tx;
    LOG_D(PHY,"[UE %d] frame %d, nr_slot_rx %d, Received PBCH (MIB): frame_tx %d. N_RB_DL %d\n",
    ue->Mod_id,
    frame_rx,
    nr_slot_rx,
    frame_tx,
    ue->frame_parms.N_RB_DL);
#endif

  } else {
    LOG_E(PHY,"[UE %d] frame %d, nr_slot_rx %d, Error decoding PBCH!\n",
	  ue->Mod_id,frame_rx, nr_slot_rx);
    /*FILE *fd;
    if ((fd = fopen("rxsig_frame0.dat","w")) != NULL) {
                  fwrite((void *)&ue->common_vars.rxdata[0][0],
                         sizeof(int32_t),
                         ue->frame_parms.samples_per_frame,
                         fd);
                  LOG_I(PHY,"Dummping Frame ... bye bye \n");
                  fclose(fd);
                  exit(0);
                }*/

    /*
    write_output("rxsig0.m","rxs0", ue->common_vars.rxdata[0],ue->frame_parms.samples_per_subframe,1,1);


      write_output("H00.m","h00",&(ue->common_vars.dl_ch_estimates[0][0][0]),((ue->frame_parms.Ncp==0)?7:6)*(ue->frame_parms.ofdm_symbol_size),1,1);
      write_output("H10.m","h10",&(ue->common_vars.dl_ch_estimates[0][2][0]),((ue->frame_parms.Ncp==0)?7:6)*(ue->frame_parms.ofdm_symbol_size),1,1);

      write_output("rxsigF0.m","rxsF0", ue->common_vars.rxdataF[0],8*ue->frame_parms.ofdm_symbol_size,1,1);
      exit(-1);
    */

  }


  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_PBCH_PROCEDURES, VCD_FUNCTION_OUT);
}



unsigned int nr_get_tx_amp(int power_dBm, int power_max_dBm, int N_RB_UL, int nb_rb)
{

  int gain_dB = power_dBm - power_max_dBm;
  double gain_lin;

  gain_lin = pow(10,.1*gain_dB);
  if ((nb_rb >0) && (nb_rb <= N_RB_UL)) {
    return((int)(AMP*sqrt(gain_lin*N_RB_UL/(double)nb_rb)));
  }
  else {
    LOG_E(PHY,"Illegal nb_rb/N_RB_UL combination (%d/%d)\n",nb_rb,N_RB_UL);
    //mac_xface->macphy_exit("");
  }
  return(0);
}

/* To be called every slot after PDSCH scheduled */
void nr_pdsch_slot_init(const nr_phy_data_t *phyData,
                        PHY_VARS_NR_UE *ue)
{
  /* checking if re-initialization of scrambling IDs is needed */
  const NR_UE_DLSCH_t *dlsch0 = &phyData->dlsch[0];
  const int scramblingId     = dlsch0->dlsch_config.dlDmrsScramblingId;
  const int nscid             = dlsch0->dlsch_config.nscid;
  if (scramblingId != ue->scramblingID_dlsch[nscid]) {
    ue->scramblingID_dlsch[nscid] = scramblingId;
    nr_gold_pdsch(ue, nscid, scramblingId);
  }
}

/* Generate channel estimates for received OFDM symbol */
int nr_pdsch_generate_channel_estimates(const PHY_VARS_NR_UE *ue,
                                        const UE_nr_rxtx_proc_t *proc,
                                        const int symbol,
                                        const NR_UE_DLSCH_t *dlsch,
                                        const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size],
                                        c16_t channel_estimates[dlsch->Nl][ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size])
{
  if (!get_isPilot_symbol(symbol, dlsch)) return -1;
  const bool is_SI            = dlsch->rnti_type == _SI_RNTI_;
  const int BWPStart          = dlsch->dlsch_config.BWPStart;
  const int pdschStartRb    = dlsch->dlsch_config.start_rb;
  const int pdschNbRb       = dlsch->dlsch_config.number_rbs;
  const int nbAntRx           = ue->frame_parms.nb_antennas_rx;
  int retVal = 0;

  /* TODO: Could be launched in Tpool for MIMO */
  for (int l = 0; l < dlsch->Nl; l++) {//for MIMO Config: it shall loop over no_layers
    for (int aarx = 0; aarx < nbAntRx; aarx++) {
      const int port = get_dmrs_port(l, dlsch->dlsch_config.dmrs_ports);
      retVal += nr_pdsch_channel_estimation(ue,
                                            proc,
                                            is_SI,
                                            port,
                                            aarx,
                                            symbol,
                                            BWPStart,
                                            dlsch->dlsch_config.dmrsConfigType,
                                            ue->frame_parms.first_carrier_offset+(BWPStart + pdschStartRb)*12,
                                            pdschNbRb,
                                            rxdataF[aarx],
                                            channel_estimates[l][aarx]);
    }
  }
  return 0;
}

/* Extract data resourse elements from received OFDM symbol */
void nr_extract_data_res(const NR_DL_FRAME_PARMS *frame_parms,
                         const fapi_nr_dl_config_dlsch_pdu_rel15_t *dlsch_config,
                         const bool isPilot,
                         const c16_t rxdataF[frame_parms->ofdm_symbol_size],
                         c16_t rxdataF_ext[dlsch_config->number_rbs * NR_NB_SC_PER_RB])
{
  const int pdschStartRb    = dlsch_config->start_rb;
  const int pdschNbRb       = dlsch_config->number_rbs;
  const int startRe = (frame_parms->first_carrier_offset + pdschStartRb * NR_NB_SC_PER_RB) % frame_parms->ofdm_symbol_size;
  if (isPilot) {
    const int configType     = dlsch_config->dmrsConfigType;
    const int nDmrsCdmGroups = dlsch_config->n_dmrs_cdm_groups;
    if (configType == NFAPI_NR_DMRS_TYPE1) {
      if (nDmrsCdmGroups == 1) {
        int k = startRe;
        for (int j = 0; j < 6*pdschNbRb; j += 3) {
          rxdataF_ext[j]   = rxdataF[k+1];
          rxdataF_ext[j+1] = rxdataF[k+3];
          rxdataF_ext[j+2] = rxdataF[k+5];
          k += 6;
          if (k >= frame_parms->ofdm_symbol_size) {
            k -= frame_parms->ofdm_symbol_size;
          }
        }
      }
    } else { /* configType == NFAPI_NR_DMRS_TYPE2 */
      if (nDmrsCdmGroups == 1) {
        int k = startRe;
        for (int j = 0; j < 8*pdschNbRb; j += 4) {
          rxdataF_ext[j]   = rxdataF[k+2];
          rxdataF_ext[j+1] = rxdataF[k+3];
          rxdataF_ext[j+2] = rxdataF[k+4];
          rxdataF_ext[j+3] = rxdataF[k+5];
          k += 6;
          if (k >= frame_parms->ofdm_symbol_size) {
            k -= frame_parms->ofdm_symbol_size;
          }
        }
      } else if (nDmrsCdmGroups == 2) {
        int k = startRe;
        for (int j = 0; j < 4*pdschNbRb; j += 2) {
          rxdataF_ext[j]   = rxdataF[k+4];
          rxdataF_ext[j+1] = rxdataF[k+5];
          k += 6;
          if (k >= frame_parms->ofdm_symbol_size) {
            k -= frame_parms->ofdm_symbol_size;
          }
        }
      }
    }
  } else {
    if (pdschStartRb + pdschNbRb * NR_NB_SC_PER_RB <= frame_parms->ofdm_symbol_size) {
      memcpy(rxdataF_ext, &rxdataF[startRe], pdschNbRb * NR_NB_SC_PER_RB * sizeof(int32_t));
    } else {
      const int negLength = frame_parms->ofdm_symbol_size - startRe;
      const int posLength = pdschNbRb * NR_NB_SC_PER_RB - negLength;
      memcpy(rxdataF_ext, &rxdataF[startRe], negLength * sizeof(int32_t));
      memcpy(&rxdataF_ext[negLength], rxdataF, posLength * sizeof(int32_t));
    }
  }
}

/* Extract data resourse elements from received OFDM symbol for all antennas */
void nr_generate_pdsch_extracted_rxdataF(const PHY_VARS_NR_UE *ue,
                                         const UE_nr_rxtx_proc_t *proc,
                                         const int symbol,
                                         const NR_UE_DLSCH_t *dlsch,
                                         const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size],
                                         c16_t rxdataF_ext[ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  /* TODO: Could be launched in Tpool for MIMO */
  for (int aarx = 0; aarx < ue->frame_parms.nb_antennas_rx; aarx++) {
    nr_extract_data_res(&ue->frame_parms,
                        &dlsch->dlsch_config,
                        get_isPilot_symbol(symbol, dlsch),
                        rxdataF[aarx],
                        rxdataF_ext[aarx]);
  }
}

/* Extract channel estimates for corresponding data REs in a OFDM symbol */
void nr_extract_pdsch_chest_res(const NR_DL_FRAME_PARMS *frame_parms,
                                const fapi_nr_dl_config_dlsch_pdu_rel15_t *dlsch_config,
                                const bool isPilot,
                                const c16_t dl_ch_est[frame_parms->ofdm_symbol_size],
                                c16_t dl_ch_est_ext[dlsch_config->number_rbs * NR_NB_SC_PER_RB])
{
  const int pdschNbRb = dlsch_config->number_rbs;
  if (isPilot) {
    const int configType     = dlsch_config->dmrsConfigType;
    const int nDmrsCdmGroups = dlsch_config->n_dmrs_cdm_groups;
    if (configType == NFAPI_NR_DMRS_TYPE1) {
      if (nDmrsCdmGroups == 1) {
        for (int j = 0; j < 6*pdschNbRb; j += 3) {
          dl_ch_est_ext[j]   = dl_ch_est[1];
          dl_ch_est_ext[j+1] = dl_ch_est[3];
          dl_ch_est_ext[j+2] = dl_ch_est[5];
          dl_ch_est += 6;
        }
      }
    } else { /* configType == NFAPI_NR_DMRS_TYPE2 */
      if (nDmrsCdmGroups == 1) {
        for (int j = 0; j < 8*pdschNbRb; j += 4) {
          dl_ch_est_ext[j]   = dl_ch_est[1];
          dl_ch_est_ext[j+1] = dl_ch_est[3];
          dl_ch_est_ext[j+2] = dl_ch_est[4];
          dl_ch_est_ext[j+3] = dl_ch_est[5];
          dl_ch_est += 6;
        }
      } else if (nDmrsCdmGroups == 2) {
        for (int j = 0; j < 4*pdschNbRb; j += 2) {
          dl_ch_est_ext[j]   = dl_ch_est[4];
          dl_ch_est_ext[j+1] = dl_ch_est[5];
          dl_ch_est += 6;
        }
      }
    }
  } else {
    memcpy(dl_ch_est_ext, dl_ch_est, pdschNbRb * NR_NB_SC_PER_RB * sizeof(int32_t));
  }
}

/* Do time domain averaging of received channel estimates for all antennas */
void nr_pdsch_estimates_time_avg(const NR_UE_DLSCH_t *dlsch,
                                 const NR_DL_FRAME_PARMS *frame_parms,
                                 c16_t dl_ch_est[NR_SYMBOLS_PER_SLOT]
                                                [frame_parms->nb_antennas_rx]
                                                [dlsch->Nl]
                                                [frame_parms->ofdm_symbol_size])
{
  const int nbAntRx = frame_parms->nb_antennas_rx;
  const int dlDmrsSymbPos = dlsch->dlsch_config.dlDmrsSymbPos;
  /* TODO: Could be launched in Tpool for MIMO */
  for (int aarx = 0; aarx < nbAntRx; aarx++) {
    for (int l = 0; l < dlsch->Nl; l++) {
      /* Average estimates in time if configured */
      nr_chest_time_domain_avg(frame_parms,
                               dlsch->dlsch_config.number_symbols,
                               dlsch->dlsch_config.start_symbol,
                               dlDmrsSymbPos,
                               dlsch->dlsch_config.number_rbs,
                               aarx,
                               l,
                               dlsch->Nl,
                               dl_ch_est);
    }
  }
}

/* Extract channel estimates for corresponding data REs in a OFDM symbol for all antennas */
void nr_generate_pdsch_extracted_chestimates(const PHY_VARS_NR_UE *ue,
                                             const NR_UE_DLSCH_t *dlsch,
                                             const int symbol,
                                             const c16_t dl_ch_est[NR_SYMBOLS_PER_SLOT]
                                                                  [dlsch->Nl]
                                                                  [ue->frame_parms.nb_antennas_rx]
                                                                  [ue->frame_parms.ofdm_symbol_size],
                                             c16_t dl_ch_est_ext[dlsch->Nl]
                                                                [ue->frame_parms.nb_antennas_rx]
                                                                [dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  const NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
  const int nbAntRx = frame_parms->nb_antennas_rx;
  const int dlDmrsSymbPos = dlsch->dlsch_config.dlDmrsSymbPos;
  /* TODO: Could be launched in Tpool for MIMO */
  const int validDmrsEst = (ue->chest_time == 0) ?
                           get_valid_dmrs_idx_for_channel_est(dlDmrsSymbPos, symbol) :
                           get_next_dmrs_symbol_in_slot(dlDmrsSymbPos, 0, 14);
  for (int aarx = 0; aarx < nbAntRx; aarx++) {
    for (int l = 0; l < dlsch->Nl; l++) {
      /* Extract estimates for all symbols */
      nr_extract_pdsch_chest_res(frame_parms,
                                 &dlsch->dlsch_config,
                                 get_isPilot_symbol(symbol, dlsch),
                                 dl_ch_est[validDmrsEst][l][aarx],
                                 dl_ch_est_ext[l][aarx]);
    }
  }
}

/* Do channel scaling on all antennas */
void nr_pdsch_channel_level_scaling(const NR_DL_FRAME_PARMS *frame_parms,
                                    const NR_UE_DLSCH_t *dlsch,
                                    const int symbol,
                                    c16_t dl_ch_est_ext[dlsch->Nl]
                                                       [frame_parms->nb_antennas_rx]
                                                       [dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  const int nl = dlsch->Nl;
  const int nbAntRx = frame_parms->nb_antennas_rx;
  for (int l = 0; l < nl; l++) {
    for (int aarx = 0; aarx < nbAntRx; aarx++) {
      nr_scale_channel(get_nb_re_pdsch_symbol(symbol, dlsch),
                       dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB,
                       dl_ch_est_ext[l][aarx]);
    }
  }
}

/* Get average channel level from all antennas */
int32_t get_maxh_extimates(const NR_DL_FRAME_PARMS *frame_parms,
                           const NR_UE_DLSCH_t *dlsch,
                           const int symbol,
                           const c16_t dl_ch_est_ext[dlsch->Nl]
                                                    [frame_parms->nb_antennas_rx]
                                                    [dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  const int nl = dlsch->Nl;
  const int nbAntRx = frame_parms->nb_antennas_rx;
  int avg;
  int median;
  int avgs = 0;
  for (int l = 0; l < nl; l++) {
    for (int aarx = 0; aarx < nbAntRx; aarx++) {
      avg = get_nr_channel_level(get_nb_re_pdsch_symbol(symbol, dlsch),
                                 dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB,
                                 dl_ch_est_ext[l][aarx]);
      avgs = cmax(avgs, avg);
      if (nl > 1) {
        median = get_nr_channel_level_median((const int)avg,
                                             get_nb_re_pdsch_symbol(symbol, dlsch),
                                             dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB,
                                             dl_ch_est_ext[l][aarx]);
        avgs = cmax(avgs, median);
      }
    }
  }
  return ( (log2_approx(avgs)/2) + 1 );
}

/* Do channel compenstion on all antennas */
void nr_pdsch_channel_compensation(const NR_DL_FRAME_PARMS *frame_parms,
                                   const NR_UE_DLSCH_t *dlsch,
                                   const int symbol,
                                   const c16_t rxdataF_ext[frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   const c16_t dl_ch_est_ext[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   c16_t dl_ch_mag[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   c16_t dl_ch_magb[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   c16_t dl_ch_magr[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   c16_t rxdataF_comp[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  for (int aarx = 0; aarx < frame_parms->nb_antennas_rx; aarx++) {
    for (int l = 0; l < dlsch->Nl; l++) {
      nr_channel_compensation(get_nb_re_pdsch_symbol(symbol, dlsch),
                              dlsch->dlsch_config.number_rbs,
                              get_maxh_extimates(frame_parms, dlsch, symbol, dl_ch_est_ext),
                              dlsch->dlsch_config.qamModOrder,
                              dl_ch_est_ext[l][aarx],
                              rxdataF_ext[aarx],
                              dl_ch_mag[l][aarx],
                              dl_ch_magb[l][aarx],
                              dl_ch_magr[l][aarx],
                              rxdataF_comp[l][aarx]);
    }
  }
}

/* Do several channel measurements */
void nr_pdsch_measurements(const NR_DL_FRAME_PARMS *frame_parms,
                           const NR_UE_DLSCH_t *dlsch,
                           const int symbol,
                           const int output_shift,
                           const c16_t dl_ch_est_ext[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                           PHY_NR_MEASUREMENTS *measurements)
{
  for (int aarx = 0; aarx < frame_parms->nb_antennas_rx; aarx++) {
    /* Channel correlation matrix */
    int rho[dlsch->Nl][dlsch->Nl][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB];
    nr_compute_channel_correlation(dlsch->Nl,
                                   get_nb_re_pdsch_symbol(symbol, dlsch),
                                   dlsch->dlsch_config.number_rbs,
                                   frame_parms->nb_antennas_rx,
                                   aarx,
                                   output_shift,
                                   dl_ch_est_ext,
                                   rho);
    if (symbol == get_first_symb_idx_with_data(dlsch->dlsch_config.dlDmrsSymbPos,
                                               dlsch->dlsch_config.dmrsConfigType,
                                               dlsch->dlsch_config.n_dmrs_cdm_groups,
                                               dlsch->dlsch_config.start_symbol,
                                               dlsch->dlsch_config.number_symbols)) {
      for (int l = 0; l < dlsch->Nl; l++) {
        for (int atx = 0; atx < dlsch->Nl; atx++) {
          measurements->rx_correlation[0][aarx][l * dlsch->Nl + atx] = signal_energy(rho[aarx][l][atx], get_nb_re_pdsch_symbol(symbol, dlsch));
        }
      }
    } // First symbol with data
  } // Antennas
}

/* Do PTRS estimation on all antennas */
void nr_pdsch_ptrs_processing(const PHY_VARS_NR_UE *ue,
                              const int gNB_id,
                              const int rnti,
                              const int slot,
                              const int symbol,
                              const NR_UE_DLSCH_t *dlsch,
                              c16_t rxdataF_comp[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                              c16_t ptrs_phase[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT],
                              int32_t ptrs_re[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT])
{
  for (int aarx = 0; aarx < ue->frame_parms.nb_antennas_rx; aarx++) {
    nr_pdsch_ptrs_processing_core(ue,
                                  gNB_id,
                                  slot,
                                  symbol,
                                  get_nb_re_pdsch_symbol(symbol, dlsch),
                                  rnti,
                                  dlsch,
                                  rxdataF_comp[0][aarx],
                                  ptrs_phase[aarx] + symbol,
                                  ptrs_re[aarx] + symbol);
  }
}

void nr_pdsch_comp_out(void *parms)
{
  nr_ue_symb_data_t *msg = (nr_ue_symb_data_t *)parms;
  const PHY_VARS_NR_UE *ue = msg->UE;
  const UE_nr_rxtx_proc_t *proc = msg->proc;
  const NR_UE_DLSCH_t *dlsch = &msg->phy_data->dlsch[0];
  const int symbolVecSize = dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB;
  const c16_t (*dl_ch_est)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                          [ue->frame_parms.ofdm_symbol_size] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                    [ue->frame_parms.ofdm_symbol_size])msg->pdsch_dl_ch_estimates;

  const c16_t (*dl_ch_est_ext)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                              [symbolVecSize] = 
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                    [symbolVecSize])msg->pdsch_dl_ch_est_ext;

  nr_generate_pdsch_extracted_chestimates(ue, dlsch, msg->symbol, *dl_ch_est, *dl_ch_est_ext);
  nr_pdsch_channel_level_scaling(&ue->frame_parms, dlsch, msg->symbol, *dl_ch_est_ext);
  const int maxh = get_maxh_extimates(&ue->frame_parms, dlsch, msg->symbol, *dl_ch_est_ext);

  const c16_t (*rxdataF_ext)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                            [symbolVecSize] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                    [symbolVecSize])msg->rxdataF_ext;

  c16_t (*dl_ch_mag)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                    [symbolVecSize] =
    (c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
              [symbolVecSize])msg->dl_ch_mag;

  c16_t (*dl_ch_magb)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                     [symbolVecSize] =
    (c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
              [symbolVecSize])msg->dl_ch_magb;

  c16_t (*dl_ch_magr)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                     [symbolVecSize] =
    (c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
              [symbolVecSize])msg->dl_ch_magr;

  c16_t (*rxdataF_comp)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                       [symbolVecSize] =
    (c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
              [symbolVecSize])msg->rxdataF_comp;

  nr_pdsch_channel_compensation(&ue->frame_parms,
                                dlsch,
                                msg->symbol,
                                *rxdataF_ext,
                                *dl_ch_est_ext,
                                *dl_ch_mag,
                                *dl_ch_magb,
                                *dl_ch_magr,
                                *rxdataF_comp);
  if (ue->frame_parms.nb_antennas_rx > 1) {
    const int nb_re_pdsch = get_nb_re_pdsch_symbol(msg->symbol, dlsch);
    nr_dlsch_detection_mrc(dlsch->Nl, ue->frame_parms.nb_antennas_rx, dlsch->dlsch_config.number_rbs, nb_re_pdsch, *rxdataF_comp, NULL, *dl_ch_mag, *dl_ch_magb, *dl_ch_magr);
    if (dlsch->Nl >= 2) {
      nr_zero_forcing_rx(dlsch->Nl,
                         ue->frame_parms.nb_antennas_rx,
                         dlsch->dlsch_config.number_rbs,
                         nb_re_pdsch,
                         dlsch->dlsch_config.qamModOrder,
                         maxh,
                         *dl_ch_est_ext,
                         *rxdataF_comp,
                         NULL,
                         *dl_ch_mag,
                         *dl_ch_magb,
                         *dl_ch_magr);
    }
  }

  c16_t (*ptrs_phase)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT] =
    (c16_t (*)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT])msg->ptrs_phase_per_slot;

  int32_t (*ptrs_re)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT] =
    (int32_t (*)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT])msg->ptrs_re_per_slot;

  if((dlsch->dlsch_config.pduBitmap & 0x1) && (dlsch->rnti_type == _C_RNTI_)) {
    nr_pdsch_ptrs_processing(ue, proc->gNB_id, dlsch->rnti, proc->nr_slot_rx, msg->symbol, dlsch, *rxdataF_comp, *ptrs_phase, *ptrs_re);
  }
}

void nr_ue_pdsch_procedures_symbol(void *params)
{
  nr_ue_symb_data_t *symbMsg = (nr_ue_symb_data_t *)params;
  PHY_VARS_NR_UE *ue = symbMsg->UE;
  UE_nr_rxtx_proc_t *proc = symbMsg->proc;
  nr_phy_data_t *phy_data = symbMsg->phy_data;
  NR_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

  start_meas(&symbMsg->pdsch_pre_proc);

  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  int gNB_id = proc->gNB_id;
  int symbol = symbMsg->symbol;
  LOG_D(PHY, "In %s symbol %d\n", __FUNCTION__, symbol);

  c16_t (*ptrs_phase_per_slot)[][NR_SYMBOLS_PER_SLOT] = symbMsg->ptrs_phase_per_slot;
  int32_t (*ptrs_re_per_slot)[][NR_SYMBOLS_PER_SLOT] = symbMsg->ptrs_re_per_slot;
  c16_t ***rxdataF_comp = (*symbMsg->rxdataF_comp)[symbol];
  c16_t **pdsch_dl_ch_estimates = (*symbMsg->pdsch_dl_ch_estimates)[symbol];
  int32_t ***dl_ch_mag  = (*symbMsg->dl_ch_mag)[symbol];
  int32_t ***dl_ch_magb = (*symbMsg->dl_ch_magb)[symbol];
  int32_t ***dl_ch_magr = (*symbMsg->dl_ch_magr)[symbol];

  NR_UE_DLSCH_t *dlsch0 = &symbMsg->phy_data->dlsch[0];

  /* dlsch should be active */
  DevAssert(dlsch0->active);

  /* We handle only one CW now */
  DevAssert(!(NR_MAX_NB_LAYERS>4));

  if (gNB_id > 2) {
    LOG_W(PHY, "In %s: Illegal gNB_id %d\n", __FUNCTION__, gNB_id);
    return;
  }

  int harq_pid = dlsch0->dlsch_config.harq_process_nbr;
  NR_DL_UE_HARQ_t *dlsch0_harq = &ue->dl_harq_processes[0][harq_pid];
  uint16_t BWPStart       = dlsch0->dlsch_config.BWPStart;
  uint16_t pdsch_start_rb = dlsch0->dlsch_config.start_rb;
  uint16_t pdsch_nb_rb    = dlsch0->dlsch_config.number_rbs;
  uint16_t s0             = dlsch0->dlsch_config.start_symbol;
  uint16_t s1             = dlsch0->dlsch_config.number_symbols;
  bool is_SI              = dlsch0->rnti_type == _SI_RNTI_;
  int nl                  = dlsch0->Nl;
  int dlDmrsSymbPos       = dlsch0->dlsch_config.dlDmrsSymbPos;
  const bool isPilot      = get_isPilot_symbol(symbol, dlsch0);

  LOG_D(PHY,"[UE %d] nr_slot_rx %d, harq_pid %d (%d), rb_start %d, nb_rb %d, symbol_start %d, nb_symbols %d, DMRS mask %x, Nl %d\n",
        ue->Mod_id,nr_slot_rx,harq_pid,dlsch0_harq->status,pdsch_start_rb,pdsch_nb_rb,s0,s1,dlsch0->dlsch_config.dlDmrsSymbPos, dlsch0->Nl);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_SLOT_FEP_PDSCH, VCD_FUNCTION_OUT);

  //----------------------------------------------------------
  //--------------------- Channel estimation -----------------
  //----------------------------------------------------------
  if (isPilot) {
    for (uint8_t aatx=0; aatx<nl; aatx++) {//for MIMO Config: it shall loop over no_layers
      LOG_D(PHY,"PDSCH Channel estimation gNB id %d, PDSCH antenna port %d, slot %d, symbol %d\n",0,aatx,nr_slot_rx,symbol);
      nr_pdsch_channel_estimation(ue,
                                  proc,
                                  is_SI,
                                  get_dmrs_port(aatx,dlsch0->dlsch_config.dmrs_ports),
                                  symbol,
                                  dlsch0->dlsch_config.nscid,
                                  dlsch0->dlsch_config.dlDmrsScramblingId,
                                  BWPStart,
                                  dlsch0->dlsch_config.dmrsConfigType,
                                  frame_parms->first_carrier_offset+(BWPStart + pdsch_start_rb)*12,
                                  pdsch_nb_rb,
                                  pdsch_dl_ch_estimates,
                                  ue->common_vars.rxdataF[symbol]);
    }
  }

  const int start_rb = dlsch0->dlsch_config.start_rb;
  const int nb_rb_pdsch =  dlsch0->dlsch_config.number_rbs;

  //----------------------------------------------------------
  //--------------------- RBs extraction ---------------------
  //----------------------------------------------------------
  const uint8_t config_type = dlsch0->dlsch_config.dmrsConfigType;
  const int n_rx = frame_parms->nb_antennas_rx;
  const uint32_t rx_size_symbol = dlsch0->dlsch_config.number_rbs * NR_NB_SC_PER_RB;
  __attribute__((aligned(32))) c16_t rxdataF_ext[n_rx][rx_size_symbol];
  memset(rxdataF_ext, 0, sizeof(rxdataF_ext));

  const int matrixSz = frame_parms->nb_antennas_rx * nl;
  __attribute__((aligned(32))) c16_t dl_ch_estimates_ext[matrixSz][rx_size_symbol];
  memset(dl_ch_estimates_ext, 0, sizeof(dl_ch_estimates_ext));

  int validDmrsEst;
  if (ue->chest_time == 1) {
    /* time average estimates are stored in first DMRS symbol */
    validDmrsEst = get_next_dmrs_symbol_in_slot(dlDmrsSymbPos,0,14);
  }
  else
    validDmrsEst = get_valid_dmrs_idx_for_channel_est(dlDmrsSymbPos,symbol);

  nr_dlsch_extract_rbs(ue->common_vars.rxdataF[symbol],
                       (*symbMsg->pdsch_dl_ch_estimates)[validDmrsEst],
                       rx_size_symbol,
                       rxdataF_ext,
                       dl_ch_estimates_ext,
                       symbol,
                       isPilot,
                       config_type,
                       start_rb + dlsch0->dlsch_config.BWPStart,
                       nb_rb_pdsch,
                       dlsch0->dlsch_config.n_dmrs_cdm_groups,
                       nl,
                       frame_parms,
                       dlsch0->dlsch_config.dlDmrsSymbPos);

  if (ue->phy_sim_pdsch_rxdataF_ext) {
    int offset = (symbol * rx_size_symbol);
    memcpy(((c16_t*)ue->phy_sim_pdsch_rxdataF_ext) + offset, rxdataF_ext[0], rx_size_symbol * sizeof(c16_t));
  }

  //----------------------------------------------------------
  //--------------------- Channel Scaling --------------------
  //----------------------------------------------------------
  const int nb_re_pdsch = get_nb_re_pdsch_symbol(symbol, dlsch0);
  nr_dlsch_scale_channel(rx_size_symbol, dl_ch_estimates_ext, frame_parms, nl, n_rx, symbol, isPilot, nb_re_pdsch, nb_rb_pdsch);

  //----------------------------------------------------------
  //--------------------- Channel Level Calc. ----------------
  //----------------------------------------------------------
  int32_t log2_maxh = 0;
  int avgs = 0;
  int avg[16];
  int32_t median[16];

  if (nb_re_pdsch > 0) {
    nr_dlsch_channel_level(rx_size_symbol, dl_ch_estimates_ext, frame_parms, nl, avg, symbol, nb_re_pdsch, nb_rb_pdsch);
    avgs = 0;
    for (int aatx=0;aatx<nl;aatx++) {
      for (int aarx=0;aarx<n_rx;aarx++) {
        avgs = cmax(avgs,avg[(aatx*n_rx)+aarx]);
        median[(aatx*n_rx)+aarx] = avg[(aatx*n_rx)+aarx];
      }
    }
    if (nl > 1) {
      nr_dlsch_channel_level_median(rx_size_symbol, dl_ch_estimates_ext, median, nl, n_rx, nb_re_pdsch);
      for (int aatx = 0; aatx < nl; aatx++) {
        for (int aarx = 0; aarx < n_rx; aarx++) {
          avgs = cmax(avgs, median[aatx*n_rx + aarx]);
        }
      }
    }
    log2_maxh = (log2_approx(avgs)/2) + 1;
    LOG_D(PHY, "[DLSCH] AbsSubframe %d.%d log2_maxh = %d (%d,%d)\n", frame_rx % 1024, nr_slot_rx, log2_maxh, avg[0], avgs);
    #if T_TRACER
    T(T_UE_PHY_PDSCH_ENERGY, T_INT(gNB_id),  T_INT(0), T_INT(frame_rx%1024), T_INT(nr_slot_rx),
      T_INT(avg[0]), T_INT(avg[1]), T_INT(avg[2]), T_INT(avg[3]));
    #endif
  }

  //----------------------------------------------------------
  //--------------------- channel compensation ---------------
  //----------------------------------------------------------
  // Disable correlation measurement for optimizing UE
  uint32_t dmrs_data_re;
  bool isFirstDataSymbol = false;

  if (dlsch0->dlsch_config.dmrsConfigType == NFAPI_NR_DMRS_TYPE1)
    dmrs_data_re = 12 - 6 * dlsch0->dlsch_config.n_dmrs_cdm_groups;
  else
    dmrs_data_re = 12 - 4 * dlsch0->dlsch_config.n_dmrs_cdm_groups;

  if ((dmrs_data_re == 0) && (!isPilot)) isFirstDataSymbol = true;

  nr_dlsch_channel_compensation(rx_size_symbol,
                                n_rx,
                                rxdataF_ext,
                                dl_ch_estimates_ext,
                                dl_ch_mag,
                                dl_ch_magb,
                                dl_ch_magr,
                                rxdataF_comp,
                                NULL,
                                frame_parms,
                                nl,
                                symbol,
                                nb_re_pdsch,
                                isFirstDataSymbol,
                                dlsch0->dlsch_config.qamModOrder,
                                nb_rb_pdsch,
                                log2_maxh,
                                &ue->measurements);

  if (ue->phy_sim_pdsch_rxdataF_comp)
    for (int a = 0; a < n_rx; a++) {
      int offset = sizeof(c16_t) * (symbol * dlsch0->Nl * n_rx * rx_size_symbol + (a * rx_size_symbol));
      memcpy(ue->phy_sim_pdsch_rxdataF_comp + offset, rxdataF_comp[0][a], sizeof(c16_t) * rx_size_symbol);
    }

  if (n_rx > 1) {
    nr_dlsch_detection_mrc(rx_size_symbol, nl, n_rx, rxdataF_comp, NULL, dl_ch_mag, dl_ch_magb, dl_ch_magr, nb_rb_pdsch, nb_re_pdsch);
    if (nl >= 2)//Apply zero forcing for 2, 3, and 4 Tx layers
      nr_zero_forcing_rx(
          rx_size_symbol, n_rx, nl, rxdataF_comp, dl_ch_mag, dl_ch_magb, dl_ch_magr, dl_ch_estimates_ext, nb_rb_pdsch, dlsch0->dlsch_config.qamModOrder, log2_maxh, nb_re_pdsch);
  }

  /* set PTRS bitmap */
  const int pduBitmap = dlsch0->dlsch_config.pduBitmap;
  if ((symbol == get_next_dmrs_symbol_in_slot(dlDmrsSymbPos, 0, NR_SYMBOLS_PER_SLOT)) && /* first DMRS symbol = first symbol processed in slot */
      (pduBitmap & 0x1)) {
    dlsch0->ptrs_symbols = get_ptrs_symb_idx(s1,
                                             s0,
                                             1 << dlsch0->dlsch_config.PTRSTimeDensity,
                                             dlDmrsSymbPos);
  }
  /* Check for PTRS bitmap and process it respectively */
  if((pduBitmap & 0x1) && (dlsch0->rnti_type == _C_RNTI_)) {
    nr_pdsch_ptrs_processing(
      ue, ue->frame_parms.nb_antennas_rx, ptrs_phase_per_slot, ptrs_re_per_slot, rxdataF_comp, &ue->frame_parms, dlsch0_harq, NULL, gNB_id, nr_slot_rx, symbol, (pdsch_nb_rb * 12), dlsch0->rnti, phy_data->dlsch);
  }

  if (ue->phy_sim_pdsch_dl_ch_estimates_ext)
    memcpy(ue->phy_sim_pdsch_dl_ch_estimates_ext + symbol * sizeof(dl_ch_estimates_ext), dl_ch_estimates_ext, sizeof(dl_ch_estimates_ext));

  stop_meas(&symbMsg->pdsch_pre_proc);
}

/* Performs for a OFDM symbol:
  1) PTRS phase compensation
  2) LLR generation for all layers
*/
void pdsch_llr_generation(const NR_DL_FRAME_PARMS *frame_parms,
                          const int symbol,
                          const NR_UE_DLSCH_t *dlsch,
                          const c16_t ptrs_phase[frame_parms->nb_antennas_rx][NR_SYMBOLS_PER_SLOT],
                          const int32_t ptrs_re[frame_parms->nb_antennas_rx][NR_SYMBOLS_PER_SLOT],
                          const int32_t dl_ch_mag[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                          const int32_t dl_ch_magb[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                          const int32_t dl_ch_magr[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                          const c16_t rxdataF_comp[dlsch->Nl][frame_parms->nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                          const int llrSize,
                          int16_t layerLlr[dlsch->Nl][llrSize])
{
  const int nb_re_pdsch = get_nb_re_pdsch_symbol(symbol, dlsch);
  int dl_valid_re = nb_re_pdsch;
  const int pdsch_nb_rb    = dlsch->dlsch_config.number_rbs;
  const int pduBitmap = dlsch->dlsch_config.pduBitmap;
  for (int aarx = 0; aarx < frame_parms->nb_antennas_rx; aarx++) {
    /* PTRS phase compensation */
    if((pduBitmap & 0x1) && (dlsch->rnti_type == _C_RNTI_)) {
      nr_pdsch_ptrs_compensate(ptrs_phase[aarx][symbol], symbol, dlsch, rxdataF_comp[0][aarx]);
      /* Adjust the valid DL RE's */
      if (aarx == 0) dl_valid_re -= ptrs_re[0][symbol];
    }
  }
  
  nr_dlsch_llr(frame_parms,
               dlsch,
               dl_valid_re,
               dl_ch_mag[0][0],
               dl_ch_magb[0][0],
               dl_ch_magr[0][0],
               rxdataF_comp,
               llrSize,
               layerLlr);
}

void pdsch_llr_generation_Tpool(void *parms)
{
  nr_ue_symb_data_t *msg = (nr_ue_symb_data_t *)parms;
  const PHY_VARS_NR_UE *ue = msg->UE;
  const UE_nr_rxtx_proc_t *proc = msg->proc;
  const NR_UE_DLSCH_t *dlsch = &msg->phy_data->dlsch[0];
  const c16_t (*ptrs_phase)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT] =
    (const c16_t (*)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT])msg->ptrs_phase_per_slot;
  const int32_t (*ptrs_re)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT] =
    (const int32_t (*)[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT])msg->ptrs_re_per_slot;
  const c16_t (*dl_ch_mag)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])msg->dl_ch_mag;
  const c16_t (*dl_ch_magb)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])msg->dl_ch_magb;
  const c16_t (*dl_ch_magr)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])msg->dl_ch_magr;
  const c16_t (*rxdataF_comp)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB] =
    (const c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])msg->rxdataF_comp;
  int16_t (*layerLlr)[dlsch->Nl][msg->llrSize] = (int16_t (*)[dlsch->Nl][msg->llrSize])msg->layer_llr;

  pdsch_llr_generation(ue,
                       msg->symbol,
                       dlsch,
                       *ptrs_phase,
                       *ptrs_re,
                       *dl_ch_mag,
                       *dl_ch_magb,
                       *dl_ch_magr,
                       *rxdataF_comp,
                       msg->llrSize,
                       *layerLlr);
}

/* Decode DLSCH from LLRs and send TB to MAC */
bool pdsch_post_processing(const PHY_VARS_NR_UE *ue,
                           const UE_nr_rxtx_proc_t *proc,
                           const NR_UE_DLSCH_t *dlsch,
                           const c16_t ptrs_phase[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT],
                           const int32_t ptrs_re[ue->frame_parms.nb_antennas_rx][NR_SYMBOLS_PER_SLOT],
                           const int32_t dl_ch_mag[NR_SYMBOLS_PER_SLOT][dlsch->Nl]
                                                  [ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                           const int32_t dl_ch_magb[NR_SYMBOLS_PER_SLOT][dlsch->Nl]
                                                   [ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                           const int32_t dl_ch_magr[NR_SYMBOLS_PER_SLOT][dlsch->Nl]
                                                   [ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                           const c16_t rxdataF_comp[NR_SYMBOLS_PER_SLOT][dlsch->Nl]
                                                   [ue->frame_parms.nb_antennas_rx][dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB])
{
  start_meas(&ue->pdsch_post_proc);

  const int pduBitmap = dlsch->dlsch_config.pduBitmap;
  for (int aarx = 0; aarx < ue->frame_parms.nb_antennas_rx; aarx++) {
    /* Interpolate PTRS estimated in time domain */
    if((pduBitmap & 0x1) && (dlsch->rnti_type == _C_RNTI_)) {
      nr_pdsch_ptrs_tdinterpol(dlsch, ptrs_phase[aarx]);
    }
  }

  /* FIFO to store results */
  notifiedFIFO_t resFifo;
  initNotifiedFIFO(&resFifo);

  /* create LLR layer buffer */
  const int llr_per_symbol = get_max_llr_per_symbol(dlsch);
  int16_t *layer_llr = (int16_t *)malloc16_clear(NR_SYMBOLS_PER_SLOT * NR_MAX_NB_LAYERS * llr_per_symbol * sizeof(int16_t));

  const int s0 = dlsch->dlsch_config.start_symbol;
  const int s1 = dlsch->dlsch_config.number_symbols;
  for (int j = s0; j < (s0 + s1); j++) {
    /* launch worker threads */
    notifiedFIFO_elt_t *newElt = newNotifiedFIFO_elt(sizeof(nr_ue_symb_data_t), proc->nr_slot_rx, &resFifo, pdsch_llr_generation_Tpool);
    nr_ue_symb_data_t *msg = (nr_ue_symb_data_t *) NotifiedFifoData(newElt);
    msg->symbol = j;
    msg->UE = ue;
    msg->proc = proc;
    msg->layer_llr = layer_llr + j*NR_MAX_NB_LAYERS*llr_per_symbol;
    msg->ptrs_phase_per_slot = (c16_t *)ptrs_phase;
    msg->ptrs_re_per_slot = (int32_t *)ptrs_re;
    msg->dl_ch_mag = (c16_t *)dl_ch_mag[j];
    msg->dl_ch_magb = (c16_t *)dl_ch_magb[j];
    msg->dl_ch_magr = (c16_t *)dl_ch_magr[j];
    msg->rxdataF_comp = (c16_t *)rxdataF_comp[j];
    pushTpool(&(get_nrUE_params()->Tpool), newElt);
  }

  /* Collect processed info from finished threads */
  for (int i = 0; i < s1; i++) {
    const notifiedFIFO_elt_t *res = pullTpool(&resFifo, &(get_nrUE_params()->Tpool));
    LOG_D(PHY, "Got LLRs from symbol %d\n", ((nr_ue_symb_data_t *)res->msgData)->symbol);
    if (res == NULL)
      LOG_E(PHY, "Tpool has been aborted\n");
    else
      delNotifiedFIFO_elt(res);
  }

  /* LLR buffer creation */
  uint8_t nb_re_dmrs;
  if (dlsch->dlsch_config.dmrsConfigType == NFAPI_NR_DMRS_TYPE1) {
    nb_re_dmrs = 6*dlsch->dlsch_config.n_dmrs_cdm_groups;
  }
  else {
    nb_re_dmrs = 4*dlsch->dlsch_config.n_dmrs_cdm_groups;
  }
  const int dmrs_len = get_num_dmrs(dlsch->dlsch_config.dlDmrsSymbPos);

  const int rx_llr_size = nr_get_G(dlsch->dlsch_config.number_rbs,
                                        s1,
                                        nb_re_dmrs,
                                        dmrs_len,
                                        dlsch->dlsch_config.qamModOrder,
                                        dlsch->Nl);
  const int rx_llr_buf_sz = ((rx_llr_size+15)/16)*16;
  const int nb_codewords = NR_MAX_NB_LAYERS > 4 ? 2 : 1;
  int16_t *llr = (int16_t *)malloc16_clear(nb_codewords * rx_llr_buf_sz * sizeof(int16_t));

  const int harq_pid = dlsch->dlsch_config.harq_process_nbr;
  NR_DL_UE_HARQ_t *dlsch0_harq = &ue->dl_harq_processes[0][harq_pid];
  NR_DL_UE_HARQ_t *dlsch1_harq = &ue->dl_harq_processes[1][harq_pid];

  /* LLR Layer Demapping */
  int dl_valid_re[NR_SYMBOLS_PER_SLOT];
  compute_dl_valid_re(dlsch, ptrs_re, dl_valid_re);
  nr_dlsch_layer_demapping(dlsch->Nl,
                           dlsch->dlsch_config.qamModOrder,
                           llr_per_symbol,
                           layer_llr,
                           dlsch,
                           dl_valid_re,
                           rx_llr_buf_sz,
                           llr);

  UEscopeCopy(ue, pdschLlr, llr[0], sizeof(int16_t), 1, rx_llr_size);

  stop_meas(&ue->pdsch_post_proc);
  LOG_D(PHY, "DLSCH data reception at nr_slot_rx: %d\n", proc->nr_slot_rx);

  start_meas(&ue->dlsch_procedures_stat);
  bool dec_res = nr_ue_dlsch_procedures(ue, proc, dlsch, llr);
  stop_meas(&ue->dlsch_procedures_stat);

  if (ue->phy_sim_pdsch_llr)
    memcpy(ue->phy_sim_pdsch_llr, llr[0], sizeof(int16_t)*rx_llr_buf_sz);

  /* free LLR memory */
  free(llr);
  free(layer_llr);

  return dec_res;
}

bool nr_ue_pdsch_procedures(void *parms)
{
  nr_ue_symb_data_t *msg = (nr_ue_symb_data_t *)parms;
  const PHY_VARS_NR_UE *ue = msg->UE;
  const UE_nr_rxtx_proc_t *proc = msg->proc;
  const NR_UE_DLSCH_t *dlsch = &msg->phy_data->dlsch[0];
  const int symbolVecSize = dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB;

  c16_t (*dl_ch_est)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
                    [ue->frame_parms.ofdm_symbol_size] =
    (c16_t (*)[dlsch->Nl][ue->frame_parms.nb_antennas_rx]
              [ue->frame_parms.ofdm_symbol_size])msg->pdsch_dl_ch_estimates;

  nr_pdsch_estimates_time_avg(dlsch, &ue->frame_parms, dl_ch_est);
  /* set PTRS bitmap */
  if((dlsch->dlsch_config.pduBitmap & 0x1) && (dlsch->rnti_type == _C_RNTI_)) {
    msg->phy_data->dlsch[0].ptrs_symbols = get_ptrs_symb_idx(dlsch->dlsch_config.number_symbols,
                                                             dlsch->dlsch_config.start_symbol,
                                                             1 << dlsch->dlsch_config.PTRSTimeDensity,
                                                             dlsch->dlsch_config.dlDmrsSymbPos);
  }

  notifiedFIFO_t nf;
  initNotifiedFIFO(&nf);
  for (int symbol = dlsch->dlsch_config.start_symbol;
       symbol < dlsch->dlsch_config.start_symbol + dlsch->dlsch_config.number_symbols;
       symbol++) {
    notifiedFIFO_elt_t *newElt = newNotifiedFIFO_elt(sizeof(nr_ue_symb_data_t), proc->nr_slot_tx, &nf, nr_pdsch_comp_out);
    nr_ue_symb_data_t *symbMsg = (nr_ue_symb_data_t *) NotifiedFifoData(newElt);
    const int symbBlockSize    = dlsch->Nl * ue->frame_parms.nb_antennas_rx * ue->frame_parms.ofdm_symbol_size;
    const int symbBlockSizeExt = dlsch->Nl * ue->frame_parms.nb_antennas_rx * (dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB);
    msg->UE                    = symbMsg->UE;
    msg->proc                  = symbMsg->proc;
    msg->symbol                = symbol;
    msg->pdsch_dl_ch_estimates = symbMsg->pdsch_dl_ch_estimates + (symbol * symbBlockSize);
    msg->pdsch_dl_ch_est_ext   = symbMsg->pdsch_dl_ch_est_ext + (symbol * symbBlockSizeExt);
    msg->rxdataF_ext           = symbMsg->rxdataF_ext + (symbol * symbBlockSizeExt);
    msg->rxdataF_comp          = symbMsg->rxdataF_comp + (symbol * symbBlockSizeExt);
    msg->dl_ch_mag             = symbMsg->dl_ch_mag + (symbol * symbBlockSizeExt);
    msg->dl_ch_magb            = symbMsg->dl_ch_magb + (symbol * symbBlockSizeExt);
    msg->dl_ch_magr            = symbMsg->dl_ch_magr + (symbol * symbBlockSizeExt);
    pushTpool(&(get_nrUE_params()->Tpool), newElt);
  }

  for (int resIdx = 0; resIdx < dlsch->dlsch_config.number_symbols; resIdx++) {
    const notifiedFIFO_elt_t *res = pullTpool(&nf, &(get_nrUE_params()->Tpool));
    if (res == NULL)
      LOG_E(PHY, "Tpool has been aborted\n");
    else
      delNotifiedFIFO_elt(res);
  }

  return pdsch_post_processing(msg->UE,
                               msg->proc,
                               dlsch,
                               msg->ptrs_phase_per_slot,
                               msg->ptrs_re_per_slot,
                               msg->dl_ch_mag,
                               msg->dl_ch_magb,
                               msg->dl_ch_magr,
                               msg->rxdataF_comp);
}

void nr_csi_slot_init(const PHY_VARS_NR_UE *ue,
                      const UE_nr_rxtx_proc_t *proc,
                      nr_csi_info_t *nr_csi_info,
                      nr_csi_phy_parms_t *csi_phy_parms)
{
  const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu = (fapi_nr_dl_config_csirs_pdu_rel15_t*)&ue->csirs_vars[proc->gNB_id]->csirs_config_pdu;
  nr_generate_csi_rs(&ue->frame_parms,
                     ue->nr_csi_info->csi_rs_generated_signal,
                     AMP,
                     ue->nr_csi_info,
                     (nfapi_nr_dl_tti_csi_rs_pdu_rel15_t *) csirs_config_pdu,
                     proc->nr_slot_rx,
                     csi_phy_parms);
}

void nr_ue_csirs_procedures(const PHY_VARS_NR_UE *ue,
                            const UE_nr_rxtx_proc_t *proc,
                            const nr_csi_phy_parms_t *csi_phy_parms,
                            const int symbol,
                            const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size],
                            int32_t csi_rs_ls_estimates[ue->frame_parms.nb_antennas_rx][ue->frame_parms.samples_per_slot_wCP],
                            nr_csi_symbol_res_t *csi_symb_res)
{
  int gNB_id = proc->gNB_id;

  const fapi_nr_dl_config_csirs_pdu_rel15_t *csirs_config_pdu = (fapi_nr_dl_config_csirs_pdu_rel15_t*)&ue->csirs_vars[proc->gNB_id]->csirs_config_pdu;
  nr_csi_rs_channel_estimation(ue,
                               proc,
                               csirs_config_pdu,
                               ue->nr_csi_info,
                               ue->nr_csi_info->csi_rs_generated_signal,
                               csi_phy_parms->N_cdm_groups,
                               csi_phy_parms->CDM_group_size,
                               csi_phy_parms->k_prime,
                               csi_phy_parms->l_prime,
                               csi_phy_parms->N_ports,
                               csi_phy_parms->j_cdm,
                               csi_phy_parms->k_overline,
                               csi_phy_parms->l_overline,
                               rxdataF,
                               symbol,
                               csi_rs_ls_estimates,
                               csi_phy_parms);

  /* do procedures for CSI-IM */
  if ((ue->csiim_vars[gNB_id]) && (ue->csiim_vars[gNB_id]->active == 1)) {
    nr_ue_csi_im_procedures(ue, proc, ue->common_vars.rxdataF);
    ue->csiim_vars[gNB_id]->active = 0;
  }

  /* do procedures for CSI-RS */
  if ((ue->csirs_vars[gNB_id]) && (ue->csirs_vars[gNB_id]->active == 1)) {
    nr_ue_csi_rs_procedures(ue, proc, ue->common_vars.rxdataF);
    ue->csirs_vars[gNB_id]->active = 0;
  }
}

void free_pdsch_slot_proc_buffers(nr_ue_symb_data_t *symb_data)
{
  nr_phy_data_t *phy_data = symb_data->phy_data;
  NR_DL_FRAME_PARMS *fp = &symb_data->UE->frame_parms;

  /* Free memory */
  const int pdsch_est_layer_size = fp->nb_antennas_rx * phy_data->dlsch[0].Nl;
  NR_UE_DLSCH_t *dlsch = phy_data->dlsch;
  const int pdsch_start_symbol = dlsch[0].dlsch_config.start_symbol;
  const int pdsch_num_symbols = dlsch[0].dlsch_config.number_symbols;
  for (int symbol = pdsch_start_symbol; symbol < (pdsch_start_symbol+pdsch_num_symbols); symbol++) {
    for (int i = 0; i < pdsch_est_layer_size; i++) {
      free((*symb_data->pdsch_dl_ch_estimates)[symbol][i]);
    }
    free((*symb_data->pdsch_dl_ch_estimates)[symbol]);
    for (int i = 0; i < dlsch[0].Nl; i++) {
      for (int j = 0; j < fp->nb_antennas_rx; j++) {
        free((*symb_data->dl_ch_mag)[symbol][i][j]);
        free((*symb_data->dl_ch_magb)[symbol][i][j]);
        free((*symb_data->dl_ch_magr)[symbol][i][j]);
        free((*symb_data->rxdataF_comp)[symbol][i][j]);
      }
      free((*symb_data->dl_ch_mag)[symbol][i]);
      free((*symb_data->dl_ch_magb)[symbol][i]);
      free((*symb_data->dl_ch_magr)[symbol][i]);
      free((*symb_data->rxdataF_comp)[symbol][i]);
    }
    free((*symb_data->dl_ch_mag)[symbol]);
    free((*symb_data->dl_ch_magb)[symbol]);
    free((*symb_data->dl_ch_magr)[symbol]);
    free((*symb_data->rxdataF_comp)[symbol]);
  }

}

static void push_symbol_thread(nr_ue_symb_data_t *symb_data, bool isDmrsSymbol)
{
  nr_phy_data_t *phy_data = symb_data->phy_data;
  NR_DL_FRAME_PARMS *fp = &symb_data->UE->frame_parms;
  const int symbol = symb_data->symbol;

  /* Allocate memory */
  const int pdsch_est_size = ((fp->ofdm_symbol_size + 15) / 16) * 16;
  const int pdsch_est_layer_size = fp->nb_antennas_rx * phy_data->dlsch[0].Nl;
  (*symb_data->pdsch_dl_ch_estimates)[symbol] = (c16_t**)malloc16(pdsch_est_layer_size * sizeof(c16_t*));
  for (int i = 0; i < pdsch_est_layer_size; i++) {
    (*symb_data->pdsch_dl_ch_estimates)[symbol][i] = (c16_t*)malloc16_clear(pdsch_est_size * sizeof(c16_t));
  }
  const uint32_t rx_size_symbol = phy_data->dlsch[0].dlsch_config.number_rbs * NR_NB_SC_PER_RB;
  (*symb_data->dl_ch_mag)[symbol]    = (int32_t***)malloc16(sizeof(int32_t**) * phy_data->dlsch[0].Nl);
  (*symb_data->dl_ch_magb)[symbol]   = (int32_t***)malloc16(sizeof(int32_t**) * phy_data->dlsch[0].Nl);
  (*symb_data->dl_ch_magr)[symbol]   = (int32_t***)malloc16(sizeof(int32_t**) * phy_data->dlsch[0].Nl);
  (*symb_data->rxdataF_comp)[symbol] = (c16_t***)malloc16(sizeof(c16_t**) * phy_data->dlsch[0].Nl);
  for (int i = 0; i < phy_data->dlsch[0].Nl; i++) {
    (*symb_data->dl_ch_mag)[symbol][i]    = (int32_t **)malloc16(sizeof(int32_t*) * fp->nb_antennas_rx);
    (*symb_data->dl_ch_magb)[symbol][i]   = (int32_t **)malloc16(sizeof(int32_t*) * fp->nb_antennas_rx);
    (*symb_data->dl_ch_magr)[symbol][i]   = (int32_t **)malloc16(sizeof(int32_t*) * fp->nb_antennas_rx);
    (*symb_data->rxdataF_comp)[symbol][i] = (c16_t **)malloc16(sizeof(c16_t*) * fp->nb_antennas_rx);
    for (int j = 0; j < fp->nb_antennas_rx; j++) {
      (*symb_data->dl_ch_mag)[symbol][i][j]    = (int32_t *)malloc16_clear(sizeof(int32_t) * rx_size_symbol);
      (*symb_data->dl_ch_magb)[symbol][i][j]   = (int32_t *)malloc16_clear(sizeof(int32_t) * rx_size_symbol);
      (*symb_data->dl_ch_magr)[symbol][i][j]   = (int32_t *)malloc16_clear(sizeof(int32_t) * rx_size_symbol);
      (*symb_data->rxdataF_comp)[symbol][i][j] = (c16_t *)malloc16_clear(sizeof(c16_t) * rx_size_symbol);
    }
  }

  /* set result FIFO based on symbol type */
  notifiedFIFO_t *resFifo;
  if (isDmrsSymbol)
    resFifo = symb_data->dmrsSymbProcRes;
  else
    resFifo = symb_data->symbProcRes;
  /* Start worker thread */
  notifiedFIFO_elt_t *newElt = newNotifiedFIFO_elt(sizeof(nr_ue_symb_data_t), symb_data->proc->nr_slot_rx, resFifo, nr_ue_pdsch_procedures_symbol);
  nr_ue_symb_data_t *symbMsg = (nr_ue_symb_data_t *) NotifiedFifoData(newElt);
  *symbMsg = *symb_data;
  reset_meas(&symbMsg->pdsch_pre_proc);
  pushTpool(&(get_nrUE_params()->Tpool), newElt);
}

void pdsch_symbol_proc_start(nr_ue_symb_data_t symb_data)
{
  nr_phy_data_t *phy_data = symb_data.phy_data;
  NR_DL_FRAME_PARMS *fp = &symb_data.UE->frame_parms;
  const int symbol = symb_data.symbol;
  const int dmrsSymbBitmap = phy_data->dlsch[0].dlsch_config.dlDmrsSymbPos;
  const int last_dmrs_symbol = get_last_dmrs_symbol_in_slot(dmrsSymbBitmap);
  DevAssert(last_dmrs_symbol != -1);

  if (symbol < last_dmrs_symbol) {
    if (get_isPilot_symbol(symbol, &phy_data->dlsch[0])) {
      LOG_D(PHY, "Starting PDSCH for DMRS symbol %d\n", symbol);
      push_symbol_thread(&symb_data, true);
    }
  } else if (symbol == last_dmrs_symbol) {
    /* process last DMRS symbol */
    LOG_D(PHY, "Starting PDSCH for DMRS symbol %d\n", symbol);
    push_symbol_thread(&symb_data, true);
    /* get results from processed DMRS symbol */
    int num_dmrs_symb = get_dmrs_symbols_in_slot(dmrsSymbBitmap, NR_SYMBOLS_PER_SLOT);
    for (int i = 0; i < num_dmrs_symb; i++) {
      notifiedFIFO_elt_t *res;
      res = pullTpool(symb_data.dmrsSymbProcRes, &(get_nrUE_params()->Tpool));
      if (res == NULL)
        LOG_E(PHY, "Tpool has been aborted\n");
      else
        delNotifiedFIFO_elt(res);
    }
    /* averaging channel estimates in time domain */
    /* channel estimates of all DMRS symbols should be available now */
    if (symb_data.UE->chest_time == 1) {
      nr_chest_time_domain_avg(fp,
                               (*symb_data.pdsch_dl_ch_estimates),
                               phy_data->dlsch[0].dlsch_config.number_symbols,
                               phy_data->dlsch[0].dlsch_config.start_symbol,
                               phy_data->dlsch[0].dlsch_config.dlDmrsSymbPos,
                               phy_data->dlsch[0].dlsch_config.number_rbs);
    }
    /* process skipped data symbols */
    const int last_dmrs_symbol = get_last_dmrs_symbol_in_slot(dmrsSymbBitmap);
    int data_symb_idx = last_dmrs_symbol - 1;
    while (data_symb_idx > 0) {
      const int dmrsSymbBitmap = phy_data->dlsch[0].dlsch_config.dlDmrsSymbPos;
      const bool isDataSymbol = !((dmrsSymbBitmap >> data_symb_idx) & 0x1);
      if (isDataSymbol) {
        symb_data.symbol = data_symb_idx;
        LOG_D(PHY, "Starting PDSCH for data symbol %d\n", symbol);
        push_symbol_thread(&symb_data, false);
      }
      data_symb_idx--;
    }
  } else {
    /* process current data symbol */
    LOG_D(PHY, "Starting PDSCH for data symbol %d\n", symbol);
    push_symbol_thread(&symb_data, false);
  }
}

bool pdsch_symbol_proc_end(nr_ue_symb_data_t symb_data)
{
  nr_phy_data_t *phy_data = symb_data.phy_data;
  const int dmrsSymbBitmap = phy_data->dlsch[0].dlsch_config.dlDmrsSymbPos;
  const int last_pdsch_symbol = phy_data->dlsch[0].dlsch_config.start_symbol + phy_data->dlsch[0].dlsch_config.number_symbols;
  const int dataSymbBitmap = ((1 << last_pdsch_symbol) - 1) ^ dmrsSymbBitmap;
  const int num_data_symbols = get_dmrs_symbols_in_slot(dataSymbBitmap, last_pdsch_symbol-1);
  LOG_D(PHY, "Finishing PDSCH for symbol %d\n", symb_data.symbol);
  /* Collect processed info from finished threads */
  for (int i = 0; i < num_data_symbols; i++) {
    notifiedFIFO_elt_t *res;
    res = pullTpool(symb_data.symbProcRes, &(get_nrUE_params()->Tpool));
    LOG_D(PHY, "Got result from symbol %d\n", ((nr_ue_symb_data_t*)(res->msgData))->symbol);
    merge_meas(&symb_data.UE->pdsch_pre_proc, &((nr_ue_symb_data_t*)(res->msgData))->pdsch_pre_proc);
    if (res == NULL)
      LOG_E(PHY, "Tpool has been aborted\n");
    else
      delNotifiedFIFO_elt(res);
  }

  /* Do remaining processing: PTRS compensation, LLR calculation & DLSCH decoding */
  return pdsch_post_processing(&symb_data);
}

void send_slot_ind(notifiedFIFO_t *nf, int slot) {
  if (nf) {
    notifiedFIFO_elt_t *newElt = newNotifiedFIFO_elt(sizeof(int), 0, NULL, NULL);
    int *msgData = (int *) NotifiedFifoData(newElt);
    *msgData = slot;
    pushNotifiedFIFO(nf, newElt);
  }
}

bool nr_ue_dlsch_procedures(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            NR_UE_DLSCH_t dlsch[2],
                            int16_t* llr[2]) {

  if (dlsch[0].active == false) {
    LOG_E(PHY, "DLSCH should be active when calling this function\n");
    return 1;
  }

  int gNB_id = proc->gNB_id;
  bool dec = false;
  int harq_pid = dlsch[0].dlsch_config.harq_process_nbr;
  int frame_rx = proc->frame_rx;
  int nr_slot_rx = proc->nr_slot_rx;
  uint32_t ret = UINT32_MAX, ret1 = UINT32_MAX;
  NR_DL_UE_HARQ_t *dl_harq0 = &ue->dl_harq_processes[0][harq_pid];
  NR_DL_UE_HARQ_t *dl_harq1 = &ue->dl_harq_processes[1][harq_pid];
  uint16_t dmrs_len = get_num_dmrs(dlsch[0].dlsch_config.dlDmrsSymbPos);
  nr_downlink_indication_t dl_indication;
  fapi_nr_rx_indication_t rx_ind = {0};
  uint16_t number_pdus = 1;

  uint8_t is_cw0_active = dl_harq0->status;
  uint8_t is_cw1_active = dl_harq1->status;
  uint16_t nb_symb_sch = dlsch[0].dlsch_config.number_symbols;
  uint8_t dmrs_type = dlsch[0].dlsch_config.dmrsConfigType;

  uint8_t nb_re_dmrs;
  if (dmrs_type==NFAPI_NR_DMRS_TYPE1) {
    nb_re_dmrs = 6*dlsch[0].dlsch_config.n_dmrs_cdm_groups;
  }
  else {
    nb_re_dmrs = 4*dlsch[0].dlsch_config.n_dmrs_cdm_groups;
  }

  LOG_D(PHY,"AbsSubframe %d.%d Start LDPC Decoder for CW0 [harq_pid %d] ? %d \n", frame_rx%1024, nr_slot_rx, harq_pid, is_cw0_active);
  LOG_D(PHY,"AbsSubframe %d.%d Start LDPC Decoder for CW1 [harq_pid %d] ? %d \n", frame_rx%1024, nr_slot_rx, harq_pid, is_cw1_active);

  // exit dlsch procedures as there are no active dlsch
  if (is_cw0_active != ACTIVE && is_cw1_active != ACTIVE) {
    // don't wait anymore
    const int ack_nack_slot = (proc->nr_slot_rx + dlsch[0].dlsch_config.k1_feedback) % ue->frame_parms.slots_per_frame;
    send_slot_ind(ue->tx_resume_ind_fifo[ack_nack_slot], proc->nr_slot_rx);
    return false;
  }

  // start ldpc decode for CW 0
  dl_harq0->G = nr_get_G(dlsch[0].dlsch_config.number_rbs,
                         nb_symb_sch,
                         nb_re_dmrs,
                         dmrs_len,
                         dlsch[0].dlsch_config.qamModOrder,
                         dlsch[0].Nl);

  start_meas(&ue->dlsch_unscrambling_stats);
  nr_dlsch_unscrambling(llr[0],
                        dl_harq0->G,
                        0,
                        ue->frame_parms.Nid_cell,
                        dlsch[0].rnti);
    

  stop_meas(&ue->dlsch_unscrambling_stats);

  start_meas(&ue->dlsch_decoding_stats);

  // create memory to store decoder output
  int a_segments = MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER*NR_MAX_NB_LAYERS;  //number of segments to be allocated
  int num_rb = dlsch[0].dlsch_config.number_rbs;
  if (num_rb != 273) {
    a_segments = a_segments*num_rb;
    a_segments = (a_segments/273)+1;
  }
  uint32_t dlsch_bytes = a_segments*1056;  // allocated bytes per segment
  __attribute__ ((aligned(32))) uint8_t p_b[dlsch_bytes];

  ret = nr_dlsch_decoding(ue,
                          proc,
                          gNB_id,
                          llr[0],
                          &ue->frame_parms,
                          &dlsch[0],
                          dl_harq0,
                          frame_rx,
                          nb_symb_sch,
                          nr_slot_rx,
                          harq_pid,
                          dlsch_bytes,
                          p_b);

  LOG_T(PHY,"dlsch decoding, ret = %d\n", ret);


  if(ret<ue->max_ldpc_iterations+1)
    dec = true;

  int ind_type = -1;
  switch(dlsch[0].rnti_type) {
    case _RA_RNTI_:
      ind_type = FAPI_NR_RX_PDU_TYPE_RAR;
      break;

    case _SI_RNTI_:
      ind_type = FAPI_NR_RX_PDU_TYPE_SIB;
      break;

    case _C_RNTI_:
      ind_type = FAPI_NR_RX_PDU_TYPE_DLSCH;
      break;

    default:
      AssertFatal(true, "Invalid DLSCH type %d\n", dlsch[0].rnti_type);
      break;
  }

  nr_fill_dl_indication(&dl_indication, NULL, &rx_ind, proc, ue, NULL);
  nr_fill_rx_indication(&rx_ind, ind_type, ue, &dlsch[0], NULL, number_pdus, proc, NULL, p_b);

  LOG_D(PHY, "DL PDU length in bits: %d, in bytes: %d \n", dlsch[0].dlsch_config.TBS, dlsch[0].dlsch_config.TBS / 8);

  stop_meas(&ue->dlsch_decoding_stats);
  if (cpumeas(CPUMEAS_GETSTATE))  {
    LOG_D(PHY, " --> Unscrambling for CW0 %5.3f\n",
          (ue->dlsch_unscrambling_stats.p_time)/(cpuf*1000.0));
    LOG_D(PHY, "AbsSubframe %d.%d --> LDPC Decoding for CW0 %5.3f\n",
          frame_rx%1024, nr_slot_rx,(ue->dlsch_decoding_stats.p_time)/(cpuf*1000.0));
  }

  if(is_cw1_active) {
    // start ldpc decode for CW 1
    dl_harq1->G = nr_get_G(dlsch[1].dlsch_config.number_rbs,
                           nb_symb_sch,
                           nb_re_dmrs,
                           dmrs_len,
                           dlsch[1].dlsch_config.qamModOrder,
                           dlsch[1].Nl);
    start_meas(&ue->dlsch_unscrambling_stats);
    nr_dlsch_unscrambling(llr[1],
                          dl_harq1->G,
                          0,
                          ue->frame_parms.Nid_cell,
                          dlsch[1].rnti);
    stop_meas(&ue->dlsch_unscrambling_stats);

    start_meas(&ue->dlsch_decoding_stats);

    ret1 = nr_dlsch_decoding(ue,
                             proc,
                             gNB_id,
                             llr[1],
                             &ue->frame_parms,
                             &dlsch[1],
                             dl_harq1,
                             frame_rx,
                             nb_symb_sch,
                             nr_slot_rx,
                             harq_pid,
                             dlsch_bytes,
                             p_b);
    LOG_T(PHY,"CW dlsch decoding, ret1 = %d\n", ret1);

    stop_meas(&ue->dlsch_decoding_stats);
    if (cpumeas(CPUMEAS_GETSTATE)) {
      LOG_D(PHY, " --> Unscrambling for CW1 %5.3f\n",
            (ue->dlsch_unscrambling_stats.p_time)/(cpuf*1000.0));
      LOG_D(PHY, "AbsSubframe %d.%d --> ldpc Decoding for CW1 %5.3f\n",
            frame_rx%1024, nr_slot_rx,(ue->dlsch_decoding_stats.p_time)/(cpuf*1000.0));
      }
  LOG_D(PHY, "harq_pid: %d, TBS expected dlsch1: %d \n", harq_pid, dlsch[1].dlsch_config.TBS);
  }

  //  send to mac
  if (ue->if_inst && ue->if_inst->dl_indication) {
    ue->if_inst->dl_indication(&dl_indication);
  }

  // DLSCH decoding finished! don't wait anymore
  const int ack_nack_slot = (proc->nr_slot_rx + dlsch[0].dlsch_config.k1_feedback) % ue->frame_parms.slots_per_frame;
  send_slot_ind(ue->tx_resume_ind_fifo[ack_nack_slot], proc->nr_slot_rx);

  if (ue->phy_sim_dlsch_b)
    memcpy(ue->phy_sim_dlsch_b, p_b, dlsch_bytes);

  return dec;
}

void prs_processing(PHY_VARS_NR_UE *ue,
                    UE_nr_rxtx_proc_t *proc,
                    c16_t **rxdataF[NR_SYMBOLS_PER_SLOT])
{
  int nr_slot_rx = proc->nr_slot_rx;
  int frame_rx = proc->frame_rx;
  NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  // Check for PRS slot - section 7.4.1.7.4 in 3GPP rel16 38.211
  for(int gNB_id = 0; gNB_id < ue->prs_active_gNBs; gNB_id++) {
    for(int rsc_id = 0; rsc_id < ue->prs_vars[gNB_id]->NumPRSResources; rsc_id++) {
      prs_config_t *prs_config = &ue->prs_vars[gNB_id]->prs_resource[rsc_id].prs_cfg;
      for (int i = 0; i < prs_config->PRSResourceRepetition; i++) {
        if( (((frame_rx*fp->slots_per_frame + nr_slot_rx) - (prs_config->PRSResourceSetPeriod[1] + prs_config->PRSResourceOffset) + 
               prs_config->PRSResourceSetPeriod[0])%prs_config->PRSResourceSetPeriod[0]) == i*prs_config->PRSResourceTimeGap) {
          nr_prs_channel_estimation(rsc_id,
                                    i,
                                    ue,
                                    proc,
                                    fp,
                                    rxdataF);
        }
      } // for i
    } // for rsc_id
  } // for gNB_id
}

int is_ssb_in_symbol(const int symbIdxInFrame,
                     const int slot,
                     const NR_DL_FRAME_PARMS *fp,
                     const int ssbMask,
                     const int ssbIndex)
{
  /* Skip if current SSB index is not transmitted */
  if ((ssbMask >> (31 - (ssbIndex % 32))) & 0x1) {
    return false;
  }

  const int startPbchSymb = nr_get_ssb_start_symbol(fp, ssbIndex) + 1;
  const int startPbchSymbHf = startPbchSymb + (fp->slots_per_frame * NR_SYMBOLS_PER_SLOT / 2);

  /* Skip if no SSB in current symbol */
  if (symbIdxInFrame < startPbchSymb ||
      symbIdxInFrame >= (startPbchSymb + NB_SYMBOLS_PBCH) ||
      symbIdxInFrame < startPbchSymbHf ||
      symbIdxInFrame >= (startPbchSymbHf + NB_SYMBOLS_PBCH)) {
    return false;
  }

  return true;
}

int get_ssb_index_in_symbol(const fapi_nr_config_request_t *cfg,
                            const NR_DL_FRAME_PARMS *fp,
                            const symbIdxInFrame,
                            const slot,
                            const frame)
{
  /* checking if current frame is compatible with SSB periodicity */
  if (cfg->ssb_table.ssb_period != 0 ||
      (frame%(1<<(cfg->ssb_table.ssb_period-1)))){
    return -1;
  }

  /* Find the SSB index corresponding to current symbol */
  for (int ssbIndex = 0; ssbIndex < fp->Lmax; ssbIndex++) {
    const int ssbMask = cfg->ssb_table.ssb_mask_list[ssbIndex/32].ssb_mask;
    if(is_ssb_in_symbol(symbIdxInFrame, slot, fp, ssbMask, ssbIndex)) return ssbIndex;
  }

  return -1;
}

/* Description: Generates PBCH LLRs from frequency domain signal for a OFDM symbol.
                Generates PBCH time domain channel response.
   Returns    : SSB index if symbol contains SSB. Else returns -1. */
int nr_process_pbch_symbol(PHY_VARS_NR_UE *ue,
                           const UE_nr_rxtx_proc_t *proc,
                           const int symbol,
                           const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size],
                           const int ssbIndexIn,
                           c16_t dl_ch_estimates_time[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size],
                           int16_t pbch_e_rx[NR_POLAR_PBCH_E])
{
  const fapi_nr_config_request_t *cfg = &ue->nrUE_config;
  const int symbIdxInFrame = symbol + NR_SYMBOLS_PER_SLOT * proc->nr_slot_rx;

  /* Search for SSB index if given SSB index is invalid */
  const int ssbIndex = (ssbIndexIn < 0) ? get_ssb_index_in_symbol(cfg,
                                                                  &ue->frame_parms,
                                                                  symbIdxInFrame,
                                                                  proc->nr_slot_rx,
                                                                  proc->frame_rx) : ssbIndexIn;
  if (ssbIndex < 0) return -1;
  /* Found SSB. Process it */
  const int startPbchSymb = nr_get_ssb_start_symbol(&ue->frame_parms, ssbIndex) + 1;
  const int startPbchSymbHf = startPbchSymb + (ue->frame_parms.slots_per_frame * NR_SYMBOLS_PER_SLOT / 2);
  c16_t dl_ch_estimates[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size];

  for (int aarx = 0; aarx < ue->frame_parms.nb_antennas_rx; aarx++) {
    nr_pbch_channel_estimation(ue,
                               proc,
                               (startPbchSymb%ue->frame_parms.symbols_per_slot)-1,
                               ssbIndex&7,
                               symbIdxInFrame > (ue->frame_parms.slots_per_frame * NR_SYMBOLS_PER_SLOT / 2),
                               rxdataF[aarx],
                               dl_ch_estimates);
  }

  const int relPbchSymb = (symbIdxInFrame > (ue->frame_parms.slots_per_frame * NR_SYMBOLS_PER_SLOT / 2)) ?
                          (symbIdxInFrame - startPbchSymbHf) : (symbIdxInFrame - startPbchSymb);

  nr_generate_pbch_llr(ue, relPbchSymb, ssbIndex, rxdataF, dl_ch_estimates, pbch_e_rx);
  /* Do measurements on middle symbol of PBCH block */
  if (relPbchSymb == 1) {
    nr_ue_ssb_rsrp_measurements(ue, ssbIndex, proc, rxdataF);
    nr_ue_rrc_measurements(ue, proc, rxdataF);
    /* resetting ssb index for PBCH detection if there is a stronger SSB index */
    if(ue->measurements.ssb_rsrp_dBm[ssbIndex] > ue->measurements.ssb_rsrp_dBm[ue->frame_parms.ssb_index]) {
      ue->frame_parms.ssb_index = ssbIndex;
    }
  }

  /* Get channel response to measure timing error */
  if ((ue->frame_parms.ssb_index == ssbIndex) &&
      (relPbchSymb == NB_SYMBOLS_PBCH - 1)) {
    // do ifft of channel estimate
    const idft_size_idx_t idftsizeidx = get_idft(ue->frame_parms.ofdm_symbol_size);
    idft(idftsizeidx,
    (int16_t*) &dl_ch_estimates,
    (int16_t*) dl_ch_estimates_time,
    1);

    UEscopeCopy(ue, pbchDlChEstimateTime, (void*)dl_ch_estimates_time, sizeof(struct complex16), ue->frame_parms.nb_antennas_rx, ue->frame_parms.ofdm_symbol_size);
  }

  return ssbIndex;
}

// todo:
// - power control as per 38.213 ch 7.4
void nr_ue_prach_procedures(PHY_VARS_NR_UE *ue, UE_nr_rxtx_proc_t *proc) {

  int gNB_id = proc->gNB_id;
  int frame_tx = proc->frame_tx, nr_slot_tx = proc->nr_slot_tx, prach_power; // tx_amp
  uint8_t mod_id = ue->Mod_id;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_PRACH, VCD_FUNCTION_IN);

  if (ue->prach_vars[gNB_id]->active) {

    fapi_nr_ul_config_prach_pdu *prach_pdu = &ue->prach_vars[gNB_id]->prach_pdu;
    ue->tx_power_dBm[nr_slot_tx] = prach_pdu->prach_tx_power;

    LOG_D(PHY, "In %s: [UE %d][RAPROC][%d.%d]: Generating PRACH Msg1 (preamble %d, P0_PRACH %d)\n",
          __FUNCTION__,
          mod_id,
          frame_tx,
          nr_slot_tx,
          prach_pdu->ra_PreambleIndex,
          ue->tx_power_dBm[nr_slot_tx]);

    ue->prach_vars[gNB_id]->amp = AMP;

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GENERATE_PRACH, VCD_FUNCTION_IN);

    prach_power = generate_nr_prach(ue, gNB_id, frame_tx, nr_slot_tx);

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_UE_GENERATE_PRACH, VCD_FUNCTION_OUT);

    LOG_D(PHY, "In %s: [UE %d][RAPROC][%d.%d]: Generated PRACH Msg1 (TX power PRACH %d dBm, digital power %d dBW (amp %d)\n",
      __FUNCTION__,
      mod_id,
      frame_tx,
      nr_slot_tx,
      ue->tx_power_dBm[nr_slot_tx],
      dB_fixed(prach_power),
      ue->prach_vars[gNB_id]->amp);

    ue->prach_vars[gNB_id]->active = false;
  }

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_UE_TX_PRACH, VCD_FUNCTION_OUT);

}
