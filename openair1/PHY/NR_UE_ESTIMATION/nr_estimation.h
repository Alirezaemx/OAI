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

#ifndef __NR_ESTIMATION_DEFS__H__
#define __NR_ESTIMATION_DEFS__H__


#include "PHY/defs_nr_UE.h"
//#include "PHY/defs_gNB.h"
/** @addtogroup _PHY_PARAMETER_ESTIMATION_BLOCKS_
 * @{
 */

/*!\brief Timing drift hysterisis in samples*/
#define SYNCH_HYST 2

/* A function to perform the channel estimation of DL PRS signal */
int nr_prs_channel_estimation(uint8_t rsc_id,
                              uint8_t rep_num,
                              PHY_VARS_NR_UE *ue,
                              UE_nr_rxtx_proc_t *proc,
                              NR_DL_FRAME_PARMS *frame_params,
                              c16_t **rxdataF[NR_SYMBOLS_PER_SLOT]);

/* Generic function to find the peak of channel estimation buffer */
void peak_estimator(int32_t *buffer, int32_t buf_len, int32_t *peak_idx, int32_t *peak_val);

/*!
\brief This function performs channel estimation including frequency and temporal interpolation
\param ue Pointer to UE PHY variables
\param gNB_id Index of target gNB
\param Ns slot number (0..19)
\param symbol symbol within slot
*/
void nr_pdcch_channel_estimation(const PHY_VARS_NR_UE *ue,
                                 const UE_nr_rxtx_proc_t *proc,
                                 const unsigned char symbol,
                                 const fapi_nr_coreset_t *coreset,
                                 const uint16_t first_carrier_offset,
                                 const uint16_t BWPStart,
                                 const int32_t pdcch_est_size,
                                 int32_t pdcch_dl_ch_estimates[][pdcch_est_size],
                                 const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size]);

int nr_pbch_dmrs_correlation(PHY_VARS_NR_UE *ue,
                             const UE_nr_rxtx_proc_t *proc,
                             const unsigned char symbol,
                             const int dmrss,
                             NR_UE_SSB *current_ssb,
                             const c16_t rxdataF[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size]);

int nr_pbch_channel_estimation(const PHY_VARS_NR_UE *ue,
                               const int dmrss,
                               const int ssb_index,
                               const int n_hf,
                               const c16_t rxdataF[ue->frame_parms.ofdm_symbol_size],
                               c16_t dl_ch_estimates[ue->frame_parms.nb_antennas_rx][ue->frame_parms.ofdm_symbol_size]);

int nr_pdsch_channel_estimation(const PHY_VARS_NR_UE *ue,
                                const UE_nr_rxtx_proc_t *proc,
                                const bool is_SI,
                                const unsigned int p,
                                const unsigned int aarx,
                                const unsigned char symbol,
                                const unsigned short BWPStart,
                                const uint8_t config_type,
                                const unsigned short bwp_start_subcarrier,
                                const unsigned short nb_rb_pdsch,
                                const c16_t rxdataF[ue->frame_parms.ofdm_symbol_size],
                                c16_t dl_ch_estimates[ue->frame_parms.ofdm_symbol_size]);

void nr_adjust_synch_ue(const PHY_VARS_NR_UE *ue,
                        const c16_t dl_ch_estimates_time[][ue->frame_parms.ofdm_symbol_size],
                        const uint8_t frame,
                        const uint8_t subframe,
                        const unsigned char clear,
                        const short coef,
                        int *max_pos_fil,
                        int *rx_offset,
                        int *time_sync_cell);
                      
void nr_ue_measurements(PHY_VARS_NR_UE *ue,
                        UE_nr_rxtx_proc_t *proc,
                        NR_UE_DLSCH_t *dlsch,
                        uint32_t pdsch_est_size,
                        int32_t dl_ch_estimates[][pdsch_est_size]);

void nr_ue_ssb_rsrp_measurements(PHY_VARS_NR_UE *ue,
                                 uint8_t gNB_index,
                                 UE_nr_rxtx_proc_t *proc,
                                 c16_t **rxdataF[NR_SYMBOLS_PER_SLOT]);

void nr_ue_rrc_measurements(PHY_VARS_NR_UE *ue,
                            UE_nr_rxtx_proc_t *proc,
                            c16_t **rxdataF[NR_SYMBOLS_PER_SLOT]);

void phy_adjust_gain_nr(PHY_VARS_NR_UE *ue,
                        uint32_t rx_power_fil_dB,
                        uint8_t gNB_id);

int nr_pdsch_ptrs_tdinterpol(const NR_UE_DLSCH_t *dlsch,
                             c16_t phase_per_symbol[NR_SYMBOLS_PER_SLOT]);

void nr_pdsch_ptrs_compensate(const c16_t phase_per_symbol,
                              const int symbol,
                              const NR_UE_DLSCH_t *dlsch,
                              c16_t rxdataF_comp[dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB]);

void nr_pdsch_ptrs_processing_core(const PHY_VARS_NR_UE *ue,
                                   const int gNB_id,
                                   const int nr_slot_rx,
                                   const int symbol,
                                   const int nb_re_pdsch,
                                   const int rnti,
                                   const NR_UE_DLSCH_t *dlsch,
                                   c16_t rxdataF_comp[dlsch->dlsch_config.number_rbs * NR_NB_SC_PER_RB],
                                   c16_t *phase_per_symbol,
                                   int32_t *ptrs_re_symbol);

float_t get_nr_RSRP(module_id_t Mod_id,uint8_t CC_id,uint8_t gNB_index);

#endif
