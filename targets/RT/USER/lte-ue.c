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

/*! \file lte-ue.c
 * \brief threads and support functions for real-time LTE UE target
 * \author R. Knopp, F. Kaltenberger, Navid Nikaein
 * \date 2015
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
#include "lte-softmodem.h"

#include "rt_wrapper.h"

#ifdef OPENAIR2
#include "LAYER2/MAC/defs.h"
#include "RRC/LITE/extern.h"
#endif
#include "PHY_INTERFACE/phy_stub_UE.h"
#include "PHY_INTERFACE/extern.h"

#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all
//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all

#include "PHY/extern.h"
#include "SCHED/extern.h"
#include "LAYER2/MAC/extern.h"
#include "LAYER2/MAC/proto.h"
#include <inttypes.h>
//#include "openair2/PHY_INTERFACE/phy_stub_UE.h"


#include "UTIL/LOG/log_extern.h"
#include "UTIL/OTG/otg_tx.h"
#include "UTIL/OTG/otg_externs.h"
#include "UTIL/MATH/oml.h"
#include "UTIL/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"


#include "T.h"

extern double cpuf;
extern uint8_t  nfapi_mode;

#define FRAME_PERIOD    100000000ULL
#define DAQ_PERIOD      66667ULL
#define FIFO_PRIORITY   40

typedef enum {
  pss=0,
  pbch=1,
  si=2
} sync_mode_t;

void init_UE_threads(int);
void init_UE_threads_stub(int);
void *UE_thread(void *arg);
void *UE_threadSL(void *arg);
void init_UE_stub(int nb_inst,int,int,char*,int);
void ue_stub_rx_handler(unsigned int, char *);
void init_UE(int,int,int,int,int,int,int);

int32_t **rxdata;
int32_t **txdata;

int timer_subframe;
int timer_frame;
SF_ticking *phy_stub_ticking;

#define KHz (1000UL)
#define MHz (1000*KHz)

typedef struct eutra_band_s {
  int16_t band;
  uint32_t ul_min;
  uint32_t ul_max;
  uint32_t dl_min;
  uint32_t dl_max;
  lte_frame_type_t frame_type;
} eutra_band_t;

typedef struct band_info_s {
  int nbands;
  eutra_band_t band_info[100];
} band_info_t;

band_info_t bands_to_scan;

static const eutra_band_t eutra_bands[] = {
  { 1, 1920    * MHz, 1980    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  { 2, 1850    * MHz, 1910    * MHz, 1930    * MHz, 1990    * MHz, FDD},
  { 3, 1710    * MHz, 1785    * MHz, 1805    * MHz, 1880    * MHz, FDD},
  { 4, 1710    * MHz, 1755    * MHz, 2110    * MHz, 2155    * MHz, FDD},
  { 5,  824    * MHz,  849    * MHz,  869    * MHz,  894    * MHz, FDD},
  { 6,  830    * MHz,  840    * MHz,  875    * MHz,  885    * MHz, FDD},
  { 7, 2500    * MHz, 2570    * MHz, 2620    * MHz, 2690    * MHz, FDD},
  { 8,  880    * MHz,  915    * MHz,  925    * MHz,  960    * MHz, FDD},
  { 9, 1749900 * KHz, 1784900 * KHz, 1844900 * KHz, 1879900 * KHz, FDD},
  {10, 1710    * MHz, 1770    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  {11, 1427900 * KHz, 1452900 * KHz, 1475900 * KHz, 1500900 * KHz, FDD},
  {12,  698    * MHz,  716    * MHz,  728    * MHz,  746    * MHz, FDD},
  {13,  777    * MHz,  787    * MHz,  746    * MHz,  756    * MHz, FDD},
  {14,  788    * MHz,  798    * MHz,  758    * MHz,  768    * MHz, FDD},
  {17,  704    * MHz,  716    * MHz,  734    * MHz,  746    * MHz, FDD},
  {20,  832    * MHz,  862    * MHz,  791    * MHz,  821    * MHz, FDD},
  {22, 3510    * MHz, 3590    * MHz, 3410    * MHz, 3490    * MHz, FDD},
  {33, 1900    * MHz, 1920    * MHz, 1900    * MHz, 1920    * MHz, TDD},
  {34, 2010    * MHz, 2025    * MHz, 2010    * MHz, 2025    * MHz, TDD},
  {35, 1850    * MHz, 1910    * MHz, 1850    * MHz, 1910    * MHz, TDD},
  {36, 1930    * MHz, 1990    * MHz, 1930    * MHz, 1990    * MHz, TDD},
  {37, 1910    * MHz, 1930    * MHz, 1910    * MHz, 1930    * MHz, TDD},
  {38, 2570    * MHz, 2620    * MHz, 2570    * MHz, 2630    * MHz, TDD},
  {39, 1880    * MHz, 1920    * MHz, 1880    * MHz, 1920    * MHz, TDD},
  {40, 2300    * MHz, 2400    * MHz, 2300    * MHz, 2400    * MHz, TDD},
  {41, 2496    * MHz, 2690    * MHz, 2496    * MHz, 2690    * MHz, TDD},
  {42, 3400    * MHz, 3600    * MHz, 3400    * MHz, 3600    * MHz, TDD},
  {43, 3600    * MHz, 3800    * MHz, 3600    * MHz, 3800    * MHz, TDD},
  {44, 703    * MHz, 803    * MHz, 703    * MHz, 803    * MHz, TDD},
};




pthread_t                       main_ue_thread;
pthread_attr_t                  attr_UE_thread;
struct sched_param              sched_param_UE_thread;

void phy_init_lte_ue_transport(PHY_VARS_UE *ue,int absraction_flag);

PHY_VARS_UE* init_ue_vars(LTE_DL_FRAME_PARMS *frame_parms,
			  uint8_t UE_id,
			  uint8_t abstraction_flag,
			  int sidelink_active)

{

  PHY_VARS_UE* ue;

  if (frame_parms!=(LTE_DL_FRAME_PARMS *)NULL) { // if we want to give initial frame parms, allocate the PHY_VARS_UE structure and put them in
    ue = (PHY_VARS_UE *)malloc(sizeof(PHY_VARS_UE));
    memset(ue,0,sizeof(PHY_VARS_UE));
    memcpy(&(ue->frame_parms), frame_parms, sizeof(LTE_DL_FRAME_PARMS));
  }
  else ue = PHY_vars_UE_g[UE_id][0];


  ue->Mod_id      = UE_id;
  ue->mac_enabled = 1;
  ue->sidelink_active = sidelink_active;
  
  // Panos: In phy_stub_UE (MAC-to-MAC) mode these init functions don't need to get called. Is this correct?
  if (nfapi_mode!=3)
    {
      // initialize all signal buffers
      init_lte_ue_signal(ue,1,abstraction_flag);
      // intialize transport
      init_lte_ue_transport(ue,abstraction_flag);
    }

  return(ue);
}


char uecap_xer[1024];



void init_thread(int sched_runtime, int sched_deadline, int sched_fifo, cpu_set_t *cpuset, char * name) {

#ifdef DEADLINE_SCHEDULER
  if (sched_runtime!=0) {
    struct sched_attr attr= {0};
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime  = sched_runtime;
    attr.sched_deadline = sched_deadline;
    attr.sched_period   = 0;
    AssertFatal(sched_setattr(0, &attr, 0) == 0,
		"[SCHED] %s thread: sched_setattr failed %s \n", name, strerror(errno));
    LOG_I(HW,"[SCHED][eNB] %s deadline thread %lu started on CPU %d\n",
	  name, (unsigned long)gettid(), sched_getcpu());
  }
#else
  if (CPU_COUNT(cpuset) > 0)
    AssertFatal( 0 == pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), cpuset), "");
  struct sched_param sp;
  sp.sched_priority = sched_fifo;
  AssertFatal(pthread_setschedparam(pthread_self(),SCHED_FIFO,&sp)==0,
	      "Can't set thread priority, Are you root?\n");
  /* Check the actual affinity mask assigned to the thread */
  cpu_set_t *cset=CPU_ALLOC(CPU_SETSIZE);
  if (0 == pthread_getaffinity_np(pthread_self(), CPU_ALLOC_SIZE(CPU_SETSIZE), cset)) {
    char txt[512]={0};
    for (int j = 0; j < CPU_SETSIZE; j++)
      if (CPU_ISSET(j, cset))
	sprintf(txt+strlen(txt), " %d ", j);
    printf("CPU Affinity of thread %s is %s\n", name, txt);
  }
  CPU_FREE(cset);
#endif

}

void init_UE(int nb_inst,int eMBMS_active, int uecap_xer_in, int timing_correction,int sidelink_active,int SLonly,int isSynchRef) {

  PHY_VARS_UE *UE;
  int         inst;
  int         ret;
  
  LOG_I(PHY,"UE : Calling Layer 2 for initialization\n");

  l2_init_ue(eMBMS_active,(uecap_xer_in==1)?uecap_xer:NULL,
	     0,// cba_group_active
	     0); // HO flag

  for (inst=0;inst<nb_inst;inst++) {

    LOG_I(PHY,"Initializing memory for UE instance %d (%p)\n",inst,PHY_vars_UE_g[inst]);

    PHY_vars_UE_g[inst][0] = init_ue_vars(NULL,inst,0,sidelink_active);
    // turn off timing control loop in UE
    PHY_vars_UE_g[inst][0]->no_timing_correction = timing_correction;
    
    PHY_vars_UE_g[inst][0]->SLonly = SLonly;
    PHY_vars_UE_g[inst][0]->is_SynchRef = isSynchRef;
    
    LOG_I(PHY,"Intializing UE Threads for instance %d (%p,%p)...\n",inst,PHY_vars_UE_g[inst],PHY_vars_UE_g[inst][0]);
    init_UE_threads(inst);
    UE = PHY_vars_UE_g[inst][0];

    
    if (oaisim_flag == 0) {
     
      ret = openair0_device_load(&(UE->rfdevice), &openair0_cfg[0]);
      if (ret !=0){
	exit_fun("Error loading device library");
      }
      UE->rfdevice.host_type   = RAU_HOST;
    }


    //    UE->rfdevice.type      = NONE_DEV;
    PHY_VARS_UE *UE = PHY_vars_UE_g[inst][0];
    if (UE->SLonly == 0)
      AssertFatal(0 == pthread_create(&UE->proc.pthread_ue,
				      &UE->proc.attr_ue,
				      UE_thread,
				      (void*)UE), "");
    if (UE->sidelink_active == 1)
      AssertFatal(0 == pthread_create(&UE->proc.pthread_ueSL,
				      &UE->proc.attr_ueSL,
				      UE_threadSL,
				      (void*)UE), "");
  }

  printf("UE threads created by %ld\n", gettid());
#if 0
#if defined(ENABLE_USE_MME)
  extern volatile int start_UE;
  while (start_UE == 0) {
    sleep(1);
  }
#endif
#endif
}


void init_UE_stub(int nb_inst,int eMBMS_active, int uecap_xer_in, char *emul_iface, int simL1) {

  int         inst;

  LOG_I(PHY,"UE : Calling Layer 2 for initialization\n");

  l2_init_ue(eMBMS_active,(uecap_xer_in==1)?uecap_xer:NULL,
	     0,// cba_group_active
	     0); // HO flag

  for (inst=0;inst<nb_inst;inst++) {

    LOG_I(PHY,"Initializing memory for UE instance %d (%p)\n",inst,PHY_vars_UE_g[inst]);
    PHY_vars_UE_g[inst][0] = init_ue_vars(NULL,inst,0,0);
    if (simL1 == 1) PHY_vars_UE_g[inst][0]->sidelink_l2_emulation = 2;
    else            PHY_vars_UE_g[inst][0]->sidelink_l2_emulation = 1;


  }
  init_timer_thread();

  init_sl_channel();


  for (inst=0;inst<nb_inst;inst++) {

    LOG_I(PHY,"Intializing UE Threads for instance %d (%p,%p)...\n",inst,PHY_vars_UE_g[inst],PHY_vars_UE_g[inst][0]);
    init_UE_threads_stub(inst);
  }

  printf("UE threads created \n");

  LOG_I(PHY,"Starting multicast link on %s\n",emul_iface);
  multicast_link_start(ue_stub_rx_handler,0,emul_iface);



}




/*!
 * \brief This is the UE synchronize thread.
 * It performs band scanning and synchonization.
 * \param arg is a pointer to a \ref PHY_VARS_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void *UE_thread_synch(void *arg)
{
  static int UE_thread_synch_retval;
  int i, hw_slot_offset;
  PHY_VARS_UE *UE = (PHY_VARS_UE*) arg;
  int current_band = 0;
  int current_offset = 0;
  sync_mode_t sync_mode = pbch;
  int CC_id = UE->CC_id;
  int ind;
  int found;
  int freq_offset=0;
  char threadname[128];

  UE->is_synchronized = 0;
  printf("UE_thread_sync in with PHY_vars_UE %p\n",arg);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if ( threads.iq != -1 )
    CPU_SET(threads.iq, &cpuset);
  // this thread priority must be lower that the main acquisition thread
  sprintf(threadname, "sync UE %d\n", UE->Mod_id);
  init_thread(100000, 500000, FIFO_PRIORITY-1, &cpuset, threadname);

  printf("starting UE synch thread (IC %d)\n",UE->proc.instance_cnt_synch);
  ind = 0;
  found = 0;


  if (UE->UE_scan == 0) {
    do  {
      current_band = eutra_bands[ind].band;
      printf( "Scanning band %d, dl_min %"PRIu32", ul_min %"PRIu32"\n", current_band, eutra_bands[ind].dl_min,eutra_bands[ind].ul_min);

      if ((eutra_bands[ind].dl_min <= UE->frame_parms.dl_CarrierFreq) && (eutra_bands[ind].dl_max >= UE->frame_parms.dl_CarrierFreq)) {
	for (i=0; i<4; i++)
	  uplink_frequency_offset[CC_id][i] = eutra_bands[ind].ul_min - eutra_bands[ind].dl_min;

        found = 1;
        break;
      }

      ind++;
    } while (ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]));

    if (found == 0) {
      LOG_E(PHY,"Can't find EUTRA band for frequency %d",UE->frame_parms.dl_CarrierFreq);
      exit_fun("Can't find EUTRA band for frequency");
      return &UE_thread_synch_retval;
    }


    LOG_I( PHY, "[SCHED][UE] Check absolute frequency DL %"PRIu32", UL %"PRIu32" (oai_exit %d, rx_num_channels %d)\n", UE->frame_parms.dl_CarrierFreq, UE->frame_parms.ul_CarrierFreq,oai_exit, openair0_cfg[0].rx_num_channels);

    for (i=0;i<openair0_cfg[UE->rf_map.card].rx_num_channels;i++) {
      openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = UE->frame_parms.dl_CarrierFreq;
      openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = UE->frame_parms.ul_CarrierFreq;
      openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
      if (uplink_frequency_offset[CC_id][i] != 0) //
	openair0_cfg[UE->rf_map.card].duplex_mode = duplex_mode_FDD;
      else //FDD
	openair0_cfg[UE->rf_map.card].duplex_mode = duplex_mode_TDD;
    }

    sync_mode = pbch;

  } else if  (UE->UE_scan == 1) {
    current_band=0;

    for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
      downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[CC_id].dl_min;
      uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] =
	bands_to_scan.band_info[CC_id].ul_min-bands_to_scan.band_info[CC_id].dl_min;
      openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i];
      openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] =
	downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i];
      openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;
    }
  }

  while (sync_var<0)
    pthread_cond_wait(&sync_cond, &sync_mutex);
  pthread_mutex_unlock(&sync_mutex);

  printf("Started device, unlocked sync_mutex (UE_sync_thread)\n");

  if (UE->rfdevice.trx_start_func(&UE->rfdevice) != 0 ) {
    LOG_E(HW,"Could not start the device\n");
    oai_exit=1;
  }

  while (oai_exit==0) {
    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
    while (UE->proc.instance_cnt_synch < 0)
      // the thread waits here most of the time
      pthread_cond_wait( &UE->proc.cond_synch, &UE->proc.mutex_synch );
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

    switch (sync_mode) {
    case pss:
      LOG_I(PHY,"[SCHED][UE] Scanning band %d (%d), freq %u\n",bands_to_scan.band_info[current_band].band, current_band,bands_to_scan.band_info[current_band].dl_min+current_offset);
      lte_sync_timefreq(UE,current_band,bands_to_scan.band_info[current_band].dl_min+current_offset);
      current_offset += 20000000; // increase by 20 MHz

      if (current_offset > bands_to_scan.band_info[current_band].dl_max-bands_to_scan.band_info[current_band].dl_min) {
	current_band++;
	current_offset=0;
      }

      if (current_band==bands_to_scan.nbands) {
	current_band=0;
	oai_exit=1;
      }

      for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
	downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[current_band].dl_min+current_offset;
	uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[current_band].ul_min-bands_to_scan.band_info[0].dl_min + current_offset;

	openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i];
	openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i];
	openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;
	if (UE->UE_scan_carrier) {
	  openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
	}
      }

      break;

    case pbch:

#if DISABLE_LOG_X
      printf("[UE thread Synch] Running Initial Synch (mode %d)\n",UE->mode);
#else
      LOG_I(PHY, "[UE thread Synch] Running Initial Synch (mode %d)\n",UE->mode);
#endif
      if (initial_sync( UE, UE->mode ) == 0) {

	hw_slot_offset = (UE->rx_offset<<1) / UE->frame_parms.samples_per_tti;
	LOG_I( HW, "Got synch: hw_slot_offset %d, carrier off %d Hz, rxgain %d (DL %u, UL %u), UE_scan_carrier %d\n",
	       hw_slot_offset,
	       freq_offset,
	       UE->rx_total_gain_dB,
	       downlink_frequency[0][0]+freq_offset,
	       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset,
	       UE->UE_scan_carrier );


	// rerun with new cell parameters and frequency-offset
	for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
	  openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;//-USRP_GAIN_OFFSET;
	  if (UE->UE_scan_carrier == 1) {
	    if (freq_offset >= 0)
	      openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] += abs(UE->common_vars.freq_offset);
	    else
	      openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] -= abs(UE->common_vars.freq_offset);
	    openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] =
	      openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i]+uplink_frequency_offset[CC_id][i];
	    downlink_frequency[CC_id][i] = openair0_cfg[CC_id].rx_freq[i];
	    freq_offset=0;
	  }
	}

	// reconfigure for potentially different bandwidth
	switch(UE->frame_parms.N_RB_DL) {
	case 6:
	  openair0_cfg[UE->rf_map.card].sample_rate =1.92e6;
	  openair0_cfg[UE->rf_map.card].rx_bw          =.96e6;
	  openair0_cfg[UE->rf_map.card].tx_bw          =.96e6;
	  //            openair0_cfg[0].rx_gain[0] -= 12;
	  break;
	case 25:
	  openair0_cfg[UE->rf_map.card].sample_rate =7.68e6;
	  openair0_cfg[UE->rf_map.card].rx_bw          =2.5e6;
	  openair0_cfg[UE->rf_map.card].tx_bw          =2.5e6;
	  //            openair0_cfg[0].rx_gain[0] -= 6;
	  break;
	case 50:
	  openair0_cfg[UE->rf_map.card].sample_rate =15.36e6;
	  openair0_cfg[UE->rf_map.card].rx_bw          =5.0e6;
	  openair0_cfg[UE->rf_map.card].tx_bw          =5.0e6;
	  //            openair0_cfg[0].rx_gain[0] -= 3;
	  break;
	case 100:
	  openair0_cfg[UE->rf_map.card].sample_rate=30.72e6;
	  openair0_cfg[UE->rf_map.card].rx_bw=10.0e6;
	  openair0_cfg[UE->rf_map.card].tx_bw=10.0e6;
	  //            openair0_cfg[0].rx_gain[0] -= 0;
	  break;
	}

	UE->rfdevice.trx_set_freq_func(&UE->rfdevice,&openair0_cfg[0],0);
	//UE->rfdevice.trx_set_gains_func(&openair0,&openair0_cfg[0]);
	//UE->rfdevice.trx_stop_func(&UE->rfdevice);
	sleep(1);
	init_frame_parms(&UE->frame_parms,1);
	/*if (UE->rfdevice.trx_start_func(&UE->rfdevice) != 0 ) {
	  LOG_E(HW,"Could not start the device\n");
	  oai_exit=1;
	  }*/

	if (UE->UE_scan_carrier == 1) {

	  UE->UE_scan_carrier = 0;
	} else {
	  AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
	  UE->is_synchronized = 1;
	  AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

	  if( UE->mode == rx_dump_frame ) {
	    FILE *fd;
	    if ((UE->proc.proc_rxtx[0].frame_rx&1) == 0) {  // this guarantees SIB1 is present
	      if ((fd = fopen("rxsig_frame0.dat","w")) != NULL) {
		fwrite((void*)&UE->common_vars.rxdata[0][0],
		       sizeof(int32_t),
		       10*UE->frame_parms.samples_per_tti,
		       fd);
		LOG_I(PHY,"Dummping Frame ... bye bye \n");
		fclose(fd);
		exit(0);
	      } else {
		LOG_E(PHY,"Cannot open file for writing\n");
		exit(0);
	      }
	    } else {
	      AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
	      UE->is_synchronized = 0;
	      AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

	    }
	  }
	}
      } else {
	// initial sync failed
	// calculate new offset and try again
	if (UE->UE_scan_carrier == 1) {
	  if (freq_offset >= 0)
	    freq_offset += 100;
	  freq_offset *= -1;

	  if (abs(freq_offset) > 7500) {
	    LOG_I( PHY, "[initial_sync] No cell synchronization found, abandoning\n" );
	    /*
	      FILE *fd;
	      if ((fd = fopen("rxsig_frame0.dat","w"))!=NULL) {
	      fwrite((void*)&UE->common_vars.rxdata[0][0],
	      sizeof(int32_t),
	      10*UE->frame_parms.samples_per_tti,
	      fd);
	      LOG_I(PHY,"Dummping Frame ... bye bye \n");
	      fclose(fd);
	      exit(0);
	      }

	      AssertFatal(1==0,"No cell synchronization found, abandoning");
	      return &UE_thread_synch_retval; // not reached
	    */
	  }
	}
#if DISABLE_LOG_X
	printf("[initial_sync] trying carrier off %d Hz, rxgain %d (DL %u, UL %u)\n",
	       freq_offset,
	       UE->rx_total_gain_dB,
	       downlink_frequency[0][0]+freq_offset,
	       downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset );
#else
	LOG_I(PHY, "[initial_sync] trying carrier off %d Hz, rxgain %d (DL %u, UL %u)\n",
	      freq_offset,
	      UE->rx_total_gain_dB,
	      downlink_frequency[0][0]+freq_offset,
	      downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset );
#endif

	for (i=0; i<openair0_cfg[UE->rf_map.card].rx_num_channels; i++) {
	  openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+freq_offset;
	  openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = downlink_frequency[CC_id][i]+uplink_frequency_offset[CC_id][i]+freq_offset;
	  openair0_cfg[UE->rf_map.card].rx_gain[UE->rf_map.chain+i] = UE->rx_total_gain_dB;//-USRP_GAIN_OFFSET;
	  if (UE->UE_scan_carrier==1)
	    openair0_cfg[UE->rf_map.card].autocal[UE->rf_map.chain+i] = 1;
	}
	UE->rfdevice.trx_set_freq_func(&UE->rfdevice,&openair0_cfg[0],0);
      }// initial_sync=0
      break;
    case si:
    default:
      break;
    }

    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
    // indicate readiness
    UE->proc.instance_cnt_synch--;
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_UE_THREAD_SYNCH, 0 );
  }  // while !oai_exit

  return &UE_thread_synch_retval;
}

static void *UE_thread_synchSL(void *arg)
{
  static int UE_thread_synch_retval;
  int i, hw_slot_offset;
  PHY_VARS_UE *UE = (PHY_VARS_UE*) arg;
  int current_band = 0;
  int current_offset = 0;
  sync_mode_t sync_mode = pbch;
  int CC_id = UE->CC_id;
  int ind;
  int found;
  int freq_offset=0;
  char threadname[128];

  UE->is_synchronizedSL = 0;
  printf("UE_thread_syncSL in with PHY_vars_UE %p\n",arg);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if ( threads.iq != -1 )
    CPU_SET(threads.iq, &cpuset);
  // this thread priority must be lower that the main acquisition thread
  sprintf(threadname, "syncSL UE %d\n", UE->Mod_id);
  init_thread(100000, 500000, FIFO_PRIORITY-1, &cpuset, threadname);

  printf("starting UE synchSL thread (IC %d)\n",UE->proc.instance_cnt_synchSL);
  ind = 0;
  found = 0;

  do  {
    current_band = eutra_bands[ind].band;
    printf( "Scanning band %d, dl_min %"PRIu32", ul_min %"PRIu32"\n", current_band, eutra_bands[ind].dl_min,eutra_bands[ind].ul_min);
    
    if ((eutra_bands[ind].dl_min <= UE->frame_parms.dl_CarrierFreq) && (eutra_bands[ind].dl_max >= UE->frame_parms.dl_CarrierFreq)) {
      for (i=0; i<4; i++)
	uplink_frequency_offset[CC_id][i] = eutra_bands[ind].ul_min - eutra_bands[ind].dl_min;
      
      UE->frame_parms.ul_CarrierFreq = UE->frame_parms.dl_CarrierFreq+uplink_frequency_offset[CC_id][0];
      found = 1;
      break;
    }
    
    ind++;
  } while (ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]));


  LOG_I( PHY, "[SCHED][UE] Check absolute frequency SL %"PRIu32"(oai_exit %d, rx_num_channels %d)\n", UE->frame_parms.ul_CarrierFreq, oai_exit, openair0_cfg[0].rx_num_channels);
  
  for (i=0;i<openair0_cfg[UE->rf_map.card].rx_num_channels;i++) {
    openair0_cfg[UE->rf_map.card].rx_freq[UE->rf_map.chain+i] = UE->frame_parms.ul_CarrierFreq;
    openair0_cfg[UE->rf_map.card].tx_freq[UE->rf_map.chain+i] = UE->frame_parms.ul_CarrierFreq;
    openair0_cfg[UE->rf_map.card].duplex_mode = duplex_mode_TDD;
  }

  while (sync_var<0)
    pthread_cond_wait(&sync_cond, &sync_mutex);
  pthread_mutex_unlock(&sync_mutex);




  while (oai_exit==0) {
    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synchSL), "");
    while (UE->proc.instance_cnt_synchSL < 0)
      // the thread waits here most of the time
      pthread_cond_wait( &UE->proc.cond_synchSL, &UE->proc.mutex_synchSL );
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synchSL), "");
   
    // Do initial synch here
    if (initial_syncSL(UE) >= 0)
	  
    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synchSL), "");
    UE->proc.instance_cnt_synchSL--;
    UE->is_synchronizedSL = 0;
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synchSL), "");
  }
}
/*!
 * \brief This is the UE thread for RX subframe n and TX subframe n+4.
 * This thread performs the phy_procedures_UE_RX() on every received slot.
 * then, if TX is enabled it performs TX for n+4.
 * \param arg is a pointer to a \ref PHY_VARS_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void *UE_thread_rxn_txnp4(void *arg) {
  static __thread int UE_thread_rxtx_retval;
  struct rx_tx_thread_data *rtd = arg;
  UE_rxtx_proc_t *proc = rtd->proc;
  PHY_VARS_UE    *UE   = rtd->UE;
  int ret;

  proc->instance_cnt_rxtx=-1;
  proc->subframe_rx=proc->sub_frame_start;

  char threadname[256];
  sprintf(threadname,"UE_%d_proc_%d", UE->Mod_id, proc->sub_frame_start);
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);

  if ( (proc->sub_frame_start+1)%RX_NB_TH == 0 && threads.one != -1 )
    CPU_SET(threads.one, &cpuset);
  if ( (proc->sub_frame_start+1)%RX_NB_TH == 1 && threads.two != -1 )
    CPU_SET(threads.two, &cpuset);
  if ( (proc->sub_frame_start+1)%RX_NB_TH == 2 && threads.three != -1 )
    CPU_SET(threads.three, &cpuset);
  //CPU_SET(threads.three, &cpuset);
  init_thread(900000,1000000 , FIFO_PRIORITY-1, &cpuset,
	      threadname);

  while (!oai_exit) {
    if (pthread_mutex_lock(&proc->mutex_rxtx) != 0) {
      LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
      exit_fun("nothing to add");
    }
    LOG_D(PHY,"before pthread_cond_wait : instance_cnt_rxtx %d\n",proc->instance_cnt_rxtx);
    while (proc->instance_cnt_rxtx < 0) {
      // most of the time, the thread is waiting here
      pthread_cond_wait( &proc->cond_rxtx, &proc->mutex_rxtx );
    }
    if (pthread_mutex_unlock(&proc->mutex_rxtx) != 0) {
      LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE RXn_TXnp4\n" );
      exit_fun("nothing to add");
    }

    initRefTimes(t2);
    initRefTimes(t3);
    pickTime(current);
    updateTimes(proc->gotIQs, &t2, 10000, "Delay to wake up UE_Thread_Rx (case 2)");

    // Process Rx data for one sub-frame
    lte_subframe_t sf_type = subframe_select( &UE->frame_parms, proc->subframe_rx);

    if (UE->SLonly == 0) {
      if ((sf_type == SF_DL) ||
	  (UE->frame_parms.frame_type == FDD) ||
	  (sf_type == SF_S)) {
	
	if (UE->frame_parms.frame_type == TDD) {
	  LOG_D(PHY, "%s,TDD%d,%s: calling UE_RX\n",
		threadname,
		UE->frame_parms.tdd_config,
		(sf_type==SF_DL? "SF_DL" :
		 (sf_type==SF_UL? "SF_UL" :
		  (sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
	} else {
	  LOG_D(PHY, "%s,%s,%s: calling UE_RX\n",
		threadname,
		(UE->frame_parms.frame_type==FDD? "FDD":
		 (UE->frame_parms.frame_type==TDD? "TDD":"UNKNOWN_DUPLEX_MODE")),
		(sf_type==SF_DL? "SF_DL" :
		 (sf_type==SF_UL? "SF_UL" :
		  (sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
	}
#ifdef UE_SLOT_PARALLELISATION
	phy_procedures_slot_parallelization_UE_RX( UE, proc, 0, 0, 1, UE->mode, no_relay, NULL );
#else
	phy_procedures_UE_RX( UE, proc, 0, 0, 1, UE->mode, no_relay, NULL );
#endif
      }

#if UE_TIMING_TRACE
      start_meas(&UE->generic_stat);
#endif
      if (UE->mac_enabled==1) {
	
	ret = ue_scheduler(UE->Mod_id,
			   proc->frame_rx,
			   proc->subframe_rx,
			   proc->frame_tx,
			   proc->subframe_tx,
			   subframe_select(&UE->frame_parms,proc->subframe_tx),
			   0,
			   0/*FIXME CC_id*/);
	if ( ret != CONNECTION_OK) {
	  char *txt;
	  switch (ret) {
	  case CONNECTION_LOST:
	    txt="RRC Connection lost, returning to PRACH";
	    break;
	  case PHY_RESYNCH:
	    txt="RRC Connection lost, trying to resynch";
	    break;
	  case RESYNCH:
	    txt="return to PRACH and perform a contention-free access";
	    break;
	  default:
	    txt="UNKNOWN RETURN CODE";
	  };
	  LOG_E( PHY, "[UE %"PRIu8"] Frame %"PRIu32", subframe %u %s\n",
		 UE->Mod_id, proc->frame_rx, proc->subframe_tx,txt );
	}
      }
#if UE_TIMING_TRACE
      stop_meas(&UE->generic_stat);
#endif
      
      
      // Prepare the future Tx data
      
      if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_UL) ||
	  (UE->frame_parms.frame_type == FDD) )
	if (UE->mode != loop_through_memory)
	  phy_procedures_UE_TX(UE,proc,0,0,UE->mode,no_relay);
      
      
      
      if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_S) &&
	  (UE->frame_parms.frame_type == TDD))
	if (UE->mode != loop_through_memory)
	  phy_procedures_UE_S_TX(UE,0,0,no_relay);
      updateTimes(current, &t3, 10000, "Delay to process sub-frame (case 3)");
      

    }

    // This is for Sidelink
    if (UE->sidelink_active == 1) {

      phy_procedures_UE_SL_RX(UE,proc);

      phy_procedures_UE_SL_TX(UE,proc);

    }

          if (pthread_mutex_lock(&proc->mutex_rxtx) != 0) {
	LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
	exit_fun("noting to add");
      }
      proc->instance_cnt_rxtx--;
      if (pthread_mutex_unlock(&proc->mutex_rxtx) != 0) {
	LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE RXTX\n" );
	exit_fun("noting to add");
      }
  }
  // thread finished
  free(arg);
  return &UE_thread_rxtx_retval;
}


#include "openair1/SIMULATION/TOOLS/defs.h"
unsigned int emulator_absSF;
channel_desc_t *UE2UE[NUMBER_OF_UE_MAX][NUMBER_OF_UE_MAX][MAX_NUM_CCs];

void init_sl_channel(void) {

  for (int UE_id = 0; UE_id < NB_UE_INST; UE_id++) {
    for (int UE_id2 = 1; UE_id2 < NB_UE_INST; UE_id2++) {
      UE2UE[UE_id][UE_id2][0] = 
	new_channel_desc_scm(PHY_vars_UE_g[UE_id][0]->frame_parms.nb_antennas_tx,
			     PHY_vars_UE_g[UE_id][0]->frame_parms.nb_antennas_rx,
			     AWGN, 
			     N_RB2sampling_rate(PHY_vars_UE_g[UE_id][0]->frame_parms.N_RB_UL),
			     N_RB2channel_bandwidth(PHY_vars_UE_g[UE_id][0]->frame_parms.N_RB_DL),
			     0.0,
			     0,
			     0);
      
      random_channel(UE2UE[UE_id][UE_id2][0],0);
    }
  }
}

void ue_stub_rx_handler(unsigned int num_bytes, char *rx_buffer) {

  PHY_VARS_UE *UE;
  UE = PHY_vars_UE_g[0][0];

  UE_tport_t *pdu = (UE_tport_t*)rx_buffer;
  SLSCH_t *slsch = (SLSCH_t*)&pdu->slsch;
  SLDCH_t *sldch = (SLDCH_t*)&pdu->sldch;

  switch (((UE_tport_header_t*)rx_buffer)->packet_type) {
  case TTI_SYNC:
    emulator_absSF = ((UE_tport_header_t*)rx_buffer)->absSF;
    wakeup_thread(&UE->timer_mutex,&UE->timer_cond,&UE->instance_cnt_timer,"timer_thread");
    break;
  case SLSCH:


    LOG_I(PHY,"Emulator SFN.SF %d.%d, Got SLSCH packet\n",emulator_absSF/10,emulator_absSF%10);
    LOG_I(PHY,"Received %d bytes on UE-UE link for SFN.SF %d.%d, sending SLSCH payload (%d bytes) to MAC\n",num_bytes,
	  pdu->header.absSF/10,pdu->header.absSF%10,
	  slsch->payload_length);
    printf("SLSCH:");
    for (int i=0;i<sizeof(SLSCH_t);i++) printf("%x ",((uint8_t*)slsch)[i]);
    printf("\n");

    int frame    = pdu->header.absSF/10;
    int subframe = pdu->header.absSF%10;
    if (UE->sidelink_l2_emulation == 2) {
      // do simulation here
      UE->slsch = slsch;
      check_and_generate_pscch(UE,frame,subframe);
      check_and_generate_pssch(UE,frame,subframe);
      do_SL_sig(UE2UE,subframe,&UE->frame_parms,frame,0);
      rx_slcch(UE,frame,subframe);
    }
    ue_send_sl_sdu(0,
		   0,
		   frame,subframe,
		   pdu->payload,
		   slsch->payload_length,
		   0,
		   SL_DISCOVERY_FLAG_NO);
    break;

  case SLDCH:


    LOG_I(PHY,"Emulator SFN.SF %d.%d, Got SLDCH packet\n",emulator_absSF/10,emulator_absSF%10);
    LOG_I(PHY,"Received %d bytes on UE-UE link for SFN.SF %d.%d, sending SLDCH payload (%d bytes) to MAC\n",num_bytes,
          pdu->header.absSF/10,pdu->header.absSF%10,
          sldch->payload_length);
    printf("SLDCH:");
    for (int i=0;i<sizeof(SLDCH_t);i++) printf("%x ",((uint8_t*)sldch)[i]);
    printf("\n");

    ue_send_sl_sdu(0,
                   0,
                   pdu->header.absSF/10,
                   pdu->header.absSF%10,
                   sldch->payload,
                   sldch->payload_length,
                   0,
                   SL_DISCOVERY_FLAG_YES);
    break;

  }
}

/*!
 * \brief This is the UE thread for RX subframe n and TX subframe n+4.
 * This thread performs the phy_procedures_UE_RX() on every received slot.
 * then, if TX is enabled it performs TX for n+4.
 * \param arg is a pointer to a \ref PHY_VARS_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void *UE_phy_stub_thread_rxn_txnp4(void *arg) {

  thread_top_init("UE_phy_stub_thread_rxn_txnp4",1,870000L,1000000L,1000000L);

  module_id_t Mod_id = 0;
  static __thread int UE_thread_rxtx_retval;
  struct rx_tx_thread_data *rtd = arg;
  UE_rxtx_proc_t *proc = rtd->proc;
  PHY_VARS_UE    *UE   = rtd->UE;
  int ret;
  //  double t_diff;

  char threadname[256];
  sprintf(threadname,"UE_%d_proc", UE->Mod_id);

  // Panos: Call (Sched_Rsp_t) get_nfapi_sched_response(UE->Mod_ID) to get all
  //sched_response config messages which concern the specific UE. Inside this
  //function we should somehow make the translation of rnti to Mod_ID.

  //proc->instance_cnt_rxtx=-1;

  phy_stub_ticking->ticking_var = -1;
  proc->subframe_rx=proc->sub_frame_start;

  //PANOS: CAREFUL HERE!
  wait_sync("UE_phy_stub_thread_rxn_txnp4");

  while (!oai_exit) {

    if (pthread_mutex_lock(&phy_stub_ticking->mutex_ticking) != 0) {
      LOG_E( MAC, "[SCHED][UE] error locking mutex for UE RXTX\n" );
      exit_fun("nothing to add");
    }
    while (phy_stub_ticking->ticking_var < 0) {
      // most of the time, the thread is waiting here
      //pthread_cond_wait( &proc->cond_rxtx, &proc->mutex_rxtx )
      LOG_D(MAC,"Waiting for ticking_var\n",phy_stub_ticking->ticking_var);
      pthread_cond_wait( &phy_stub_ticking->cond_ticking, &phy_stub_ticking->mutex_ticking);
    }
    phy_stub_ticking->ticking_var--;
    if (pthread_mutex_unlock(&phy_stub_ticking->mutex_ticking) != 0) {
      LOG_E( MAC, "[SCHED][UE] error unlocking mutex for UE RXn_TXnp4\n" );
      exit_fun("nothing to add");
    }
    LOG_D(MAC," Panos-D [UE_phy_stub_thread_rxn_txnp4 1] Frame: %d, Subframe: %d \n" "\n" "\n", timer_frame, timer_subframe);


    proc->subframe_rx=timer_subframe;
    proc->frame_rx = timer_frame;
    proc->subframe_tx=(timer_subframe+4)%10;
    proc->frame_tx = (proc->frame_rx + (proc->subframe_rx>5?1:0))&1023;
    //oai_subframe_ind(proc->frame_rx, proc->subframe_rx);



    // Panos: Guessing that the next 4 lines are not needed for the phy_stub mode.
    /*initRefTimes(t2);
      initRefTimes(t3);
      pickTime(current);
      updateTimes(proc->gotIQs, &t2, 10000, "Delay to wake up UE_Thread_Rx (case 2)");*/


    // Process Rx data for one sub-frame
    lte_subframe_t sf_type = subframe_select( &UE->frame_parms, proc->subframe_rx);
    if ((sf_type == SF_DL) ||
	(UE->frame_parms.frame_type == FDD) ||
	(sf_type == SF_S)) {

      if (UE->frame_parms.frame_type == TDD) {
	LOG_D(PHY, "%s,TDD%d,%s: calling UE_RX\n",
	      threadname,
	      UE->frame_parms.tdd_config,
	      (sf_type==SF_DL? "SF_DL" :
	       (sf_type==SF_UL? "SF_UL" :
		(sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
      } else {
	LOG_D(PHY, "%s,%s,%s: calling UE_RX\n",
	      threadname,
	      (UE->frame_parms.frame_type==FDD? "FDD":
	       (UE->frame_parms.frame_type==TDD? "TDD":"UNKNOWN_DUPLEX_MODE")),
	      (sf_type==SF_DL? "SF_DL" :
	       (sf_type==SF_UL? "SF_UL" :
		(sf_type==SF_S ? "SF_S"  : "UNKNOWN_SF_TYPE"))));
      }


      phy_procedures_UE_SL_RX(UE,proc);

      /*
	#ifdef UE_SLOT_PARALLELISATION
	phy_procedures_slot_parallelization_UE_RX( UE, proc, 0, 0, 1, UE->mode, no_relay, NULL );
	#else
      */
      // Panos: Substitute call to phy_procedures Rx with call to phy_stub functions in order to trigger
      // UE Rx procedures directly at the MAC layer, based on the received nfapi requests from the vnf (eNB).
      // Hardcode Mod_id for now. Will be changed later.

      // Panos: is this the right place to call oai_subframe_indication to invoke p7 nfapi callbacks here?

      //oai_subframe_ind(proc->frame_rx, proc->subframe_rx);
      //oai_subframe_ind(timer_frame, timer_subframe);

      //start_meas(&UE->timer_stats);
      //oai_subframe_ind(proc->frame_tx, proc->subframe_tx);
      oai_subframe_ind(timer_frame, timer_subframe);
      //LOG_I( MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 after oai_subframe_ind \n");
      //printf("Panos-D: UE_phy_stub_thread_rxn_txnp4 after oai_subframe_ind \n");
      /*if(UE_mac_inst[Mod_id].tx_req!= NULL){
	printf("Panos-D: UE_phy_stub_thread_rxn_txnp4 after oai_subframe_ind 2\n");
	tx_req_UE_MAC(UE_mac_inst[Mod_id].tx_req);
	}*/
      if(UE_mac_inst[Mod_id].dl_config_req!= NULL) {
	//LOG_I( MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 after oai_subframe_ind 3 \n");
	dl_config_req_UE_MAC(UE_mac_inst[Mod_id].dl_config_req);
      }
      //if(UE_mac_inst[Mod_id].hi_dci0_req!= NULL){
      if (UE_mac_inst[Mod_id].hi_dci0_req!=NULL && UE_mac_inst[Mod_id].hi_dci0_req->hi_dci0_request_body.hi_dci0_pdu_list!=NULL){
	LOG_I( MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 after oai_subframe_ind 4 \n");
	hi_dci0_req_UE_MAC(UE_mac_inst[Mod_id].hi_dci0_req);
	//if(UE_mac_inst[Mod_id].hi_dci0_req->hi_dci0_request_body.hi_dci0_pdu_list!=NULL){
	free(UE_mac_inst[Mod_id].hi_dci0_req->hi_dci0_request_body.hi_dci0_pdu_list);
	UE_mac_inst[Mod_id].hi_dci0_req->hi_dci0_request_body.hi_dci0_pdu_list = NULL;
	//}
	free(UE_mac_inst[Mod_id].hi_dci0_req);
	UE_mac_inst[Mod_id].hi_dci0_req = NULL;
      }

      else if(UE_mac_inst[Mod_id].hi_dci0_req!=NULL){
	free(UE_mac_inst[Mod_id].hi_dci0_req);
	UE_mac_inst[Mod_id].hi_dci0_req = NULL;
      }
      //stop_meas(&UE->timer_stats);
      //t_diff = get_time_meas_us(&UE->timer_stats);
      //LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
      phy_procedures_UE_SL_TX(UE,proc);
      //#endif
    }

    //>>>>>>> Stashed changes

#if UE_TIMING_TRACE
    start_meas(&UE->generic_stat);
#endif
    if (UE->mac_enabled==1) {

      ret = ue_scheduler(UE->Mod_id,
			 proc->frame_rx,
			 proc->subframe_rx,
			 proc->frame_tx,
			 proc->subframe_tx,
			 subframe_select(&UE->frame_parms,proc->subframe_tx),
			 0,
			 0/*FIXME CC_id*/);
      if ( ret != CONNECTION_OK) {
	char *txt;
	switch (ret) {
	case CONNECTION_LOST:
	  txt="RRC Connection lost, returning to PRACH";
	  break;
	case PHY_RESYNCH:
	  txt="RRC Connection lost, trying to resynch";
	  break;
	case RESYNCH:
	  txt="return to PRACH and perform a contention-free access";
	  break;
	default:
	  txt="UNKNOWN RETURN CODE";
	};
	LOG_E( PHY, "[UE %"PRIu8"] Frame %"PRIu32", subframe %u %s\n",
	       UE->Mod_id, proc->frame_rx, proc->subframe_tx,txt );
      }
    }
#if UE_TIMING_TRACE
    stop_meas(&UE->generic_stat);
#endif


    // Prepare the future Tx data

    if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_UL) ||
	(UE->frame_parms.frame_type == FDD) )
      if (UE->mode != loop_through_memory){

	if ((UE_mac_inst[Mod_id].UE_mode[0] == PRACH) ) {
	  //LOG_D(MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 before RACH \n");

	  // check if we have PRACH opportunity

	  if (is_prach_subframe(&UE->frame_parms,proc->frame_tx, proc->subframe_tx)) {
	    //LOG_I(MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 before RACH 2 \n");
	    PRACH_RESOURCES_t *prach_resources = ue_get_rach(Mod_id, 0, proc->frame_tx, 0, proc->subframe_tx);
	    if(prach_resources!=NULL) {
	      //LOG_I(MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 before RACH 3 \n");
	      fill_rach_indication_UE_MAC(Mod_id, proc->frame_tx ,proc->subframe_tx, UL_INFO, prach_resources->ra_PreambleIndex, prach_resources->ra_RNTI);
	      Msg1_transmitted(Mod_id, 0, proc->frame_tx, 0);
	      UE_mac_inst[Mod_id].UE_mode[0] = RA_RESPONSE;
	    }

	    //ue_prach_procedures(ue,proc,eNB_id,abstraction_flag,mode);
	  }
	} // mode is PRACH
	// Panos: Substitute call to phy_procedures Tx with call to phy_stub functions in order to trigger
	// UE Tx procedures directly at the MAC layer, based on the received ul_config requests from the vnf (eNB).
	// Generate UL_indications which correspond to UL traffic.
	if(UE_mac_inst[Mod_id].ul_config_req!= NULL && UE_mac_inst[Mod_id].ul_config_req->ul_config_request_body.ul_config_pdu_list != NULL){
	  //LOG_I(MAC, "Panos-D: UE_phy_stub_thread_rxn_txnp4 ul_config_req is not NULL \n");
	  ul_config_req_UE_MAC(UE_mac_inst[Mod_id].ul_config_req, timer_frame, timer_subframe);
	  //ul_config_req_UE_MAC(UE_mac_inst[Mod_id].ul_config_req, proc->frame_tx, proc->subframe_tx);
	  if(UE_mac_inst[Mod_id].ul_config_req->ul_config_request_body.ul_config_pdu_list != NULL){
	    free(UE_mac_inst[Mod_id].ul_config_req->ul_config_request_body.ul_config_pdu_list);
	    UE_mac_inst[Mod_id].ul_config_req->ul_config_request_body.ul_config_pdu_list = NULL;
	  }
	  free(UE_mac_inst[Mod_id].ul_config_req);
	  UE_mac_inst[Mod_id].ul_config_req = NULL;
	  //UL_indication(UL_INFO);
	}
	else if(UE_mac_inst[Mod_id].ul_config_req!=NULL){
	  free(UE_mac_inst[Mod_id].ul_config_req);
	  UE_mac_inst[Mod_id].ul_config_req = NULL;
	}
      }

    phy_procedures_UE_SL_RX(UE,proc);


    /*if ((subframe_select( &UE->frame_parms, proc->subframe_tx) == SF_S) &&
      (UE->frame_parms.frame_type == TDD))
      if (UE->mode != loop_through_memory)
      phy_procedures_UE_S_TX(UE,0,0,no_relay);
      updateTimes(current, &t3, 10000, "Delay to process sub-frame (case 3)");*/

    //if (pthread_mutex_lock(&proc->mutex_rxtx) != 0) {
    /*if (pthread_mutex_lock(&phy_stub_ticking->mutex_ticking) != 0) {
      LOG_E( PHY, "[SCHED][UE] error locking mutex for UE RXTX\n" );
      exit_fun("noting to add");
      }

      //proc->instance_cnt_rxtx--;

      //if (pthread_mutex_unlock(&proc->mutex_rxtx) != 0) {
      if (pthread_mutex_unlock(&phy_stub_ticking->mutex_ticking) != 0) {
      LOG_E( PHY, "[SCHED][UE] error unlocking mutex for UE RXTX\n" );
      exit_fun("noting to add");
      }*/
  }
  // thread finished
  free(arg);
  return &UE_thread_rxtx_retval;
}



/*!
 * \brief This is the main UE thread for DL/UL connectivity
 * This thread controls the other three UE threads:
 * - UE_thread_rxn_txnp4 (even subframes)
 * - UE_thread_rxn_txnp4 (odd subframes)
 * - UE_thread_synch
 * \param arg unused
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

void *UE_thread(void *arg) {


  PHY_VARS_UE *UE = (PHY_VARS_UE *) arg;
  //  int tx_enabled = 0;
  int dummy_rx[UE->frame_parms.nb_antennas_rx][UE->frame_parms.samples_per_tti] __attribute__((aligned(32)));
  openair0_timestamp timestamp,timestamp1;
  void* rxp[NB_ANTENNAS_RX], *txp[NB_ANTENNAS_TX];
  int start_rx_stream = 0;
  int i;
  int th_id;

  static uint8_t thread_idx = 0;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if ( threads.iq != -1 )
    CPU_SET(threads.iq, &cpuset);
  init_thread(100000, 500000, FIFO_PRIORITY, &cpuset,
	      "UHD Threads");

#ifdef NAS_UE
  MessageDef *message_p;
  message_p = itti_alloc_new_message(TASK_NAS_UE, INITIALIZE_MESSAGE);
  itti_send_msg_to_task (TASK_NAS_UE, UE->Mod_id + NB_eNB_INST, message_p);
#endif

  int sub_frame=-1;
  //int cumulated_shift=0;

  while (!oai_exit) {
    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
    int instance_cnt_synch = UE->proc.instance_cnt_synch;
    int is_synchronized    = UE->is_synchronized;
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

    if (is_synchronized == 0) {
      if (instance_cnt_synch < 0) {  // we can invoke the synch
	// grab 10 ms of signal and wakeup synch thread
	for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	  rxp[i] = (void*)&UE->common_vars.rxdata[i][0];
	
	if (UE->mode != loop_through_memory) 
	  AssertFatal( UE->frame_parms.samples_per_tti*10 ==
		       UE->rfdevice.trx_read_func(&UE->rfdevice,
						  &timestamp,
						  rxp,
						  UE->frame_parms.samples_per_tti*10,
						  UE->frame_parms.nb_antennas_rx), "");
	AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
	instance_cnt_synch = ++UE->proc.instance_cnt_synch;
	if (instance_cnt_synch == 0) {
	  AssertFatal( 0 == pthread_cond_signal(&UE->proc.cond_synch), "");
	} else {
	  LOG_E( PHY, "[SCHED][UE] UE sync thread busy!!\n" );
	  exit_fun("nothing to add");
	}
	AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");
      } else {
#if OAISIM
	(void)dummy_rx; /* avoid gcc warnings */
	usleep(500);
	
#else
	// grab 10 ms of signal into dummy buffer
	if (UE->mode != loop_through_memory) {
	  for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	    rxp[i] = (void*)&dummy_rx[i][0];
	  for (int sf=0; sf<10; sf++)
	    //	    printf("Reading dummy sf %d\n",sf);
	    UE->rfdevice.trx_read_func(&UE->rfdevice,
				       &timestamp,
				       rxp,
				       UE->frame_parms.samples_per_tti,
				       UE->frame_parms.nb_antennas_rx);
	}
#endif
      }// instance_cnt
	  
    } // UE->is_synchronized==0
    else {
      if (start_rx_stream==0) {
	start_rx_stream=1;
	if (UE->mode != loop_through_memory) {
	  if (UE->no_timing_correction==0) {
	    LOG_I(PHY,"Resynchronizing RX by %d samples (mode = %d)\n",UE->rx_offset,UE->mode);
	    AssertFatal(UE->rx_offset ==
			UE->rfdevice.trx_read_func(&UE->rfdevice,
						   &timestamp,
						   (void**)UE->common_vars.rxdata,
						   UE->rx_offset,
						   UE->frame_parms.nb_antennas_rx),"");
	  }
	  UE->rx_offset=0;
	  UE->time_sync_cell=0;
	  //UE->proc.proc_rxtx[0].frame_rx++;
	  //UE->proc.proc_rxtx[1].frame_rx++;
	  for (th_id=0; th_id < RX_NB_TH; th_id++) {
	    UE->proc.proc_rxtx[th_id].frame_rx++;
	  }
	    
	  // read in first symbol (only
	    
	  AssertFatal (UE->frame_parms.ofdm_symbol_size+UE->frame_parms.nb_prefix_samples0 ==
		       UE->rfdevice.trx_read_func(&UE->rfdevice,
						  &timestamp,
						  (void**)UE->common_vars.rxdata,
						  UE->frame_parms.ofdm_symbol_size+UE->frame_parms.nb_prefix_samples0,
						  UE->frame_parms.nb_antennas_rx),"");
	  slot_fep(UE,0, 0, 0, 0, 0);
	    
	} //UE->mode != loop_through_memory
	else
	  rt_sleep_ns(1000*1000);
	  
      } else {
	sub_frame++;
	sub_frame%=10;
	UE_rxtx_proc_t *proc = &UE->proc.proc_rxtx[thread_idx];
	// update thread index for received subframe
	UE->current_thread_id[sub_frame] = thread_idx;
	  
	LOG_D(PHY,"Process Subframe %d thread Idx %d \n", sub_frame, UE->current_thread_id[sub_frame]);
	  
	thread_idx++;
	if(thread_idx>=RX_NB_TH)
	  thread_idx = 0;
	  
	  
	if (UE->mode != loop_through_memory) {
	  for (i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	    rxp[i] = (void*)&UE->common_vars.rxdata[i][UE->frame_parms.ofdm_symbol_size+
						       UE->frame_parms.nb_prefix_samples0+
						       sub_frame*UE->frame_parms.samples_per_tti];
	  for (i=0; i<UE->frame_parms.nb_antennas_tx; i++)
	    txp[i] = (void*)&UE->common_vars.txdata[i][((sub_frame+2)%10)*UE->frame_parms.samples_per_tti];
	    
	  int readBlockSize, writeBlockSize;
	  if (sub_frame<9) {
	    readBlockSize=UE->frame_parms.samples_per_tti;
	    writeBlockSize=UE->frame_parms.samples_per_tti;
	  } else {
	    // set TO compensation to zero
	      
	    UE->rx_offset_diff = 0;
	      
	    // compute TO compensation that should be applied for this frame
	      
	    if (UE->no_timing_correction == 0) {
	      if ( UE->rx_offset < 5*UE->frame_parms.samples_per_tti  &&
		   UE->rx_offset > 0 )
		UE->rx_offset_diff = -1 ;
	      if ( UE->rx_offset > 5*UE->frame_parms.samples_per_tti &&
		   UE->rx_offset < 10*UE->frame_parms.samples_per_tti )
		UE->rx_offset_diff = 1;
	    }
	      
	    LOG_D(PHY,"AbsSubframe %d.%d SET rx_off_diff to %d rx_offset %d \n",proc->frame_rx,sub_frame,UE->rx_offset_diff,UE->rx_offset);
	    readBlockSize=UE->frame_parms.samples_per_tti -
	      UE->frame_parms.ofdm_symbol_size -
	      UE->frame_parms.nb_prefix_samples0 -
	      UE->rx_offset_diff;
	    writeBlockSize=UE->frame_parms.samples_per_tti -
	      UE->rx_offset_diff;
	  }
	    
	  AssertFatal(readBlockSize ==
		      UE->rfdevice.trx_read_func(&UE->rfdevice,
						 &timestamp,
						 rxp,
						 readBlockSize,
						 UE->frame_parms.nb_antennas_rx),"");
	  AssertFatal( writeBlockSize ==
		       UE->rfdevice.trx_write_func(&UE->rfdevice,
						   timestamp+
						   (2*UE->frame_parms.samples_per_tti) -
						   UE->frame_parms.ofdm_symbol_size-UE->frame_parms.nb_prefix_samples0 -
						   openair0_cfg[0].tx_sample_advance,
						   txp,
						   writeBlockSize,
						   UE->frame_parms.nb_antennas_tx,
						   1),"");
	  if( sub_frame==9) {
	    // read in first symbol of next frame and adjust for timing drift
	    int first_symbols=writeBlockSize-readBlockSize;
	    if ( first_symbols > 0 )
	      AssertFatal(first_symbols ==
			  UE->rfdevice.trx_read_func(&UE->rfdevice,
						     &timestamp1,
						     (void**)UE->common_vars.rxdata,
						     first_symbols,
						     UE->frame_parms.nb_antennas_rx),"");
	    if ( first_symbols <0 )
	      LOG_E(PHY,"can't compensate: diff =%d\n", first_symbols);
	  }
	  pickTime(gotIQs);
	  // operate on thread sf mod 2
	  AssertFatal(pthread_mutex_lock(&proc->mutex_rxtx) ==0,"");
	  if(sub_frame == 0) {
	    //UE->proc.proc_rxtx[0].frame_rx++;
	    //UE->proc.proc_rxtx[1].frame_rx++;
	    for (th_id=0; th_id < RX_NB_TH; th_id++) {
	      UE->proc.proc_rxtx[th_id].frame_rx++;
	    }
	  }
	  //UE->proc.proc_rxtx[0].gotIQs=readTime(gotIQs);
	  //UE->proc.proc_rxtx[1].gotIQs=readTime(gotIQs);
	  for (th_id=0; th_id < RX_NB_TH; th_id++) {
	    UE->proc.proc_rxtx[th_id].gotIQs=readTime(gotIQs);
	  }
	  proc->subframe_rx=sub_frame;
	  proc->subframe_tx=(sub_frame+4)%10;
	  proc->frame_tx = (proc->frame_rx + (proc->subframe_rx>5?1:0))&1023;
	  proc->timestamp_tx = timestamp+
	    (4*UE->frame_parms.samples_per_tti)-
	    UE->frame_parms.ofdm_symbol_size-UE->frame_parms.nb_prefix_samples0;
	    
	  proc->instance_cnt_rxtx++;
	  LOG_D( PHY, "[SCHED][UE %d] UE RX instance_cnt_rxtx %d subframe %d !!\n", UE->Mod_id, proc->instance_cnt_rxtx,proc->subframe_rx);
	  if (proc->instance_cnt_rxtx == 0) {
	    if (pthread_cond_signal(&proc->cond_rxtx) != 0) {
	      LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
	      exit_fun("nothing to add");
	    }
	  } else {
	    LOG_E( PHY, "[SCHED][UE %d] UE RX thread busy (IC %d)!!\n", UE->Mod_id, proc->instance_cnt_rxtx);
	    if (proc->instance_cnt_rxtx > 2)
	      exit_fun("instance_cnt_rxtx > 2");
	  }
	    
	  AssertFatal (pthread_cond_signal(&proc->cond_rxtx) ==0 ,"");
	  AssertFatal(pthread_mutex_unlock(&proc->mutex_rxtx) ==0,"");
	  initRefTimes(t1);
	  initStaticTime(lastTime);
	  updateTimes(lastTime, &t1, 20000, "Delay between two IQ acquisitions (case 1)");
	  pickStaticTime(lastTime);
	    
	} else {
	  printf("Processing subframe %d",proc->subframe_rx);
	  getchar();
	}
      } // start_rx_stream==1
    } // UE->is_synchronized==1
      
  } // while !oai_exit
  return NULL;
}


/*!
 * \brief Initialize the UE theads.
 * Creates the UE threads:
 * - UE_thread_rxtx0
 * - UE_thread_rxtx1
 * - UE_thread_synch
 * - UE_thread_fep_slot0
 * - UE_thread_fep_slot1
 * - UE_thread_dlsch_proc_slot0
 * - UE_thread_dlsch_proc_slot1
 * and the locking between them.
 */
void init_UE_threads(int inst) {
  struct rx_tx_thread_data *rtd;
  PHY_VARS_UE *UE;

  AssertFatal(PHY_vars_UE_g!=NULL,"PHY_vars_UE_g is NULL\n");
  AssertFatal(PHY_vars_UE_g[inst]!=NULL,"PHY_vars_UE_g[inst] is NULL\n");
  AssertFatal(PHY_vars_UE_g[inst][0]!=NULL,"PHY_vars_UE_g[inst][0] is NULL\n");
  UE = PHY_vars_UE_g[inst][0];

  pthread_attr_init (&UE->proc.attr_ue);
  pthread_attr_setstacksize(&UE->proc.attr_ue,8192);//5*PTHREAD_STACK_MIN);

  pthread_mutex_init(&UE->proc.mutex_synch,NULL);
  pthread_cond_init(&UE->proc.cond_synch,NULL);

  pthread_attr_init (&UE->proc.attr_ueSL);
  pthread_attr_setstacksize(&UE->proc.attr_ueSL,8192);//5*PTHREAD_STACK_MIN);

  pthread_mutex_init(&UE->proc.mutex_synchSL,NULL);
  pthread_cond_init(&UE->proc.cond_synchSL,NULL);

  // the threads are not yet active, therefore access is allowed without locking
  int nb_threads=RX_NB_TH;
  for (int i=0; i<nb_threads; i++) {
    rtd = calloc(1, sizeof(struct rx_tx_thread_data));
    if (rtd == NULL) abort();
    rtd->UE = UE;
    rtd->proc = &UE->proc.proc_rxtx[i];

    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_rxtx,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_rxtx,NULL);
    UE->proc.proc_rxtx[i].sub_frame_start=i;
    UE->proc.proc_rxtx[i].sub_frame_step=nb_threads;
    printf("Init_UE_threads rtd %d proc %d nb_threads %d i %d\n",rtd->proc->sub_frame_start, UE->proc.proc_rxtx[i].sub_frame_start,nb_threads, i);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_rxtx, NULL, UE_thread_rxn_txnp4, rtd);

#ifdef UE_SLOT_PARALLELISATION
    //pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot0_dl_processing,NULL);
    //pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot0_dl_processing,NULL);
    //pthread_create(&UE->proc.proc_rxtx[i].pthread_slot0_dl_processing,NULL,UE_thread_slot0_dl_processing, rtd);

    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot1_dl_processing,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot1_dl_processing,NULL);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_slot1_dl_processing,NULL,UE_thread_slot1_dl_processing, rtd);
#endif

  }
  if (UE->SLonly==0) pthread_create(&UE->proc.pthread_synch,NULL,UE_thread_synch,(void*)UE);
  if (UE->sidelink_active==1) {
    pthread_mutex_init(&UE->slss_mutex,NULL);
    pthread_mutex_init(&UE->sldch_mutex,NULL);
    pthread_mutex_init(&UE->slsch_mutex,NULL);
    pthread_create(&UE->proc.pthread_synchSL,NULL,UE_thread_synchSL,(void*)UE);
  }
}



/*!
 * \brief Initialize the UE theads.
 * Creates the UE threads:
 * - UE_thread_rxtx0
 * - UE_thread_synch
 * - UE_thread_fep_slot0
 * - UE_thread_fep_slot1
 * - UE_thread_dlsch_proc_slot0
 * - UE_thread_dlsch_proc_slot1
 * and the locking between them.
 */
void init_UE_threads_stub(int inst) {
  struct rx_tx_thread_data *rtd;
  PHY_VARS_UE *UE;

  AssertFatal(PHY_vars_UE_g!=NULL,"PHY_vars_UE_g is NULL\n");
  AssertFatal(PHY_vars_UE_g[inst]!=NULL,"PHY_vars_UE_g[inst] is NULL\n");
  AssertFatal(PHY_vars_UE_g[inst][0]!=NULL,"PHY_vars_UE_g[inst][0] is NULL\n");
  UE = PHY_vars_UE_g[inst][0];

  pthread_attr_init (&UE->proc.attr_ue);
  pthread_attr_setstacksize(&UE->proc.attr_ue,8192);//5*PTHREAD_STACK_MIN);

  // Panos: Don't need synch for phy_stub mode
  //pthread_mutex_init(&UE->proc.mutex_synch,NULL);
  //pthread_cond_init(&UE->proc.cond_synch,NULL);

  // the threads are not yet active, therefore access is allowed without locking
  // Panos: In phy_stub_UE mode due to less heavy processing operations we don't need two threads
  //int nb_threads=RX_NB_TH;
  int nb_threads=1;
  for (int i=0; i<nb_threads; i++) {
    rtd = calloc(1, sizeof(struct rx_tx_thread_data));
    if (rtd == NULL) abort();
    rtd->UE = UE;
    rtd->proc = &UE->proc.proc_rxtx[i];

    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_rxtx,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_rxtx,NULL);
    UE->proc.proc_rxtx[i].sub_frame_start=i;
    UE->proc.proc_rxtx[i].sub_frame_step=nb_threads;
    printf("Init_UE_threads rtd %d proc %d nb_threads %d i %d\n",rtd->proc->sub_frame_start, UE->proc.proc_rxtx[i].sub_frame_start,nb_threads, i);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_rxtx, NULL, UE_phy_stub_thread_rxn_txnp4, rtd);
    /*
      #ifdef UE_SLOT_PARALLELISATION
      //pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot0_dl_processing,NULL);
      //pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot0_dl_processing,NULL);
      //pthread_create(&UE->proc.proc_rxtx[i].pthread_slot0_dl_processing,NULL,UE_thread_slot0_dl_processing, rtd);

      pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot1_dl_processing,NULL);
      pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot1_dl_processing,NULL);
      pthread_create(&UE->proc.proc_rxtx[i].pthread_slot1_dl_processing,NULL,UE_thread_slot1_dl_processing, rtd);
      #endif*/

  }
  // Panos: Remove thread for UE_sync in phy_stub_UE mode.
  //pthread_create(&UE->proc.pthread_synch,NULL,UE_thread_synch,(void*)UE);
}


/*!
 * \brief This is the main UE thread for SL connectivity
 * This thread controls the other three UE threads:
 * - UE_thread_rxn_txnp4 (even subframes)
 * - UE_thread_rxn_txnp4 (odd subframes)
 * - UE_thread_synch
 * \param arg unused
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

void *UE_threadSL(void *arg) {


  PHY_VARS_UE *UE = (PHY_VARS_UE *) arg;
  //  int tx_enabled = 0;
  int dummy_rx[UE->frame_parms.nb_antennas_rx][UE->frame_parms.samples_per_tti] __attribute__((aligned(32)));
  openair0_timestamp timestamp,timestamp1;
  void* rxp[NB_ANTENNAS_RX], *txp[NB_ANTENNAS_TX];
  int start_rx_stream = 0;
  int i;
  int th_id;

  static uint8_t thread_idx = 0;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  if ( threads.iq != -1 )
    CPU_SET(threads.iq, &cpuset);
  init_thread(100000, 500000, FIFO_PRIORITY, &cpuset,
	      "UHD Thread SL");

#ifdef NAS_UE
  MessageDef *message_p;
  message_p = itti_alloc_new_message(TASK_NAS_UE, INITIALIZE_MESSAGE);
  itti_send_msg_to_task (TASK_NAS_UE, UE->Mod_id + NB_eNB_INST, message_p);
#endif

  int sub_frame=-1;
  //int cumulated_shift=0;

  UE->proc.instance_cnt_synchSL=-1;

  while (sync_var<0)
    pthread_cond_wait(&sync_cond, &sync_mutex);
  pthread_mutex_unlock(&sync_mutex);

  AssertFatal(UE->rfdevice.trx_start_func(&UE->rfdevice) == 0,"Could not start the device");

  
  while (!oai_exit) {
    AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synch), "");
    int instance_cnt_synch = UE->proc.instance_cnt_synchSL;
    int is_synchronized    = UE->is_synchronizedSL;
    AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synch), "");

    LOG_D(PHY,"UHD Thread SL (is_synchronized %d, is_SynchRef %d\n",
	  is_synchronized,UE->is_SynchRef);
    
    if (is_synchronized == 0 && UE->is_SynchRef == 0) {
      if (instance_cnt_synch < 0) {  // we can invoke the synch
	// grab 40 ms of signal and wakeup synch thread
	for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	  rxp[i] = (void*)&UE->common_vars.rxdata_syncSL[i][0];

	AssertFatal( UE->frame_parms.samples_per_tti*40 ==
		     UE->rfdevice.trx_read_func(&UE->rfdevice,
						&timestamp,
						rxp,
						UE->frame_parms.samples_per_tti*40,
						UE->frame_parms.nb_antennas_rx), "");
	AssertFatal ( 0== pthread_mutex_lock(&UE->proc.mutex_synchSL), "");
	instance_cnt_synch = ++UE->proc.instance_cnt_synchSL;
	if (instance_cnt_synch == 0) {
	  AssertFatal( 0 == pthread_cond_signal(&UE->proc.cond_synchSL), "");
	} else {
	  LOG_E( PHY, "[SCHED][UE] UE sync thread busy!!\n" );
	  exit_fun("nothing to add");
	}
	AssertFatal ( 0== pthread_mutex_unlock(&UE->proc.mutex_synchSL), "");
      } else {
	// grab 10 ms of signal into dummy buffer

	for (int i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	  rxp[i] = (void*)&dummy_rx[i][0];
	for (int sf=0; sf<40; sf++)
	  //	    printf("Reading dummy sf %d\n",sf);
	  UE->rfdevice.trx_read_func(&UE->rfdevice,
				     &timestamp,
				     rxp,
				     UE->frame_parms.samples_per_tti,
				     UE->frame_parms.nb_antennas_rx);
      }
      
	
    } // UE->is_synchronized==0 && UE->is_SynchRef==0
    else {
      if (start_rx_stream==0 && UE->is_SynchRef == 0) {
	start_rx_stream=1;
	if (UE->no_timing_correction==0) {
	  LOG_I(PHY,"Resynchronizing RX by %d samples (mode = %d)\n",UE->rx_offsetSL,UE->mode);
	  AssertFatal(UE->rx_offset ==
		      UE->rfdevice.trx_read_func(&UE->rfdevice,
						 &timestamp,
						 (void**)UE->common_vars.rxdata,
						 UE->rx_offset,
						 UE->frame_parms.nb_antennas_rx),"");
	}
	UE->rx_offsetSL=0;
	UE->time_sync_cell=0;
	//UE->proc.proc_rxtx[0].frame_rx++;
	//UE->proc.proc_rxtx[1].frame_rx++;
	for (th_id=0; th_id < RX_NB_TH; th_id++) {
	  UE->proc.proc_rxtx[th_id].frame_rx++;
	}
      } else { // This is steady-state mode
	sub_frame++;
	sub_frame%=10;
	UE_rxtx_proc_t *proc = &UE->proc.proc_rxtx[thread_idx];
	// update thread index for received subframe
	UE->current_thread_id[sub_frame] = thread_idx;
	
	LOG_D(PHY,"Process SL Subframe %d thread Idx %d \n", sub_frame, UE->current_thread_id[sub_frame]);
	
	thread_idx++;
	if(thread_idx>=RX_NB_TH)
	  thread_idx = 0;
	
	
	for (i=0; i<UE->frame_parms.nb_antennas_rx; i++)
	  rxp[i] = (void*)&UE->common_vars.rxdata[i][UE->frame_parms.ofdm_symbol_size+
						     UE->frame_parms.nb_prefix_samples0+
						     sub_frame*UE->frame_parms.samples_per_tti];
	for (i=0; i<UE->frame_parms.nb_antennas_tx; i++)
	  txp[i] = (void*)&UE->common_vars.txdata[i][((sub_frame+2)%10)*UE->frame_parms.samples_per_tti];
	
	int readBlockSize, writeBlockSize;
	if (sub_frame<9) {
	  readBlockSize=UE->frame_parms.samples_per_tti;
	  writeBlockSize=UE->frame_parms.samples_per_tti;
	} else {
	  // set TO compensation to zero
	  
	  if (UE->is_SynchRef==0 && UE->is_synchronized ==0) { // use timing correction from SL
	    UE->rx_offset_diff = 0;
	    
	    // compute TO compensation that should be applied for this frame
	    
	    if (UE->no_timing_correction == 0) {
	      if ( UE->rx_offsetSL < 5*UE->frame_parms.samples_per_tti  &&
		   UE->rx_offsetSL > 0 )
		UE->rx_offset_diff = -1 ;
	      if ( UE->rx_offsetSL > 5*UE->frame_parms.samples_per_tti &&
		   UE->rx_offsetSL < 10*UE->frame_parms.samples_per_tti )
		UE->rx_offset_diff = 1;
	    }
	    
	    LOG_D(PHY,"AbsSubframe %d.%d SET rx_off_diff to %d rx_offsetSL %d \n",proc->frame_rx,sub_frame,UE->rx_offset_diff,UE->rx_offsetSL);
	  } //UE->is_SynchRef==0 && uE->is_synchronized==0
	  // wait for DL thread to finish if DL synchronized
	  if (UE->is_synchronized == 1) {
	    AssertFatal(wait_on_condition(& UE->proc.mutex_SL,& UE->proc.cond_SL,& UE->proc.instance_cnt_SL,"UE_threadSL")>=0,
			"wait_on_condition failed\n");
	    release_thread(&UE->proc.mutex_SL,&UE->proc.instance_cnt_SL,"UE_threadSL");
	  }
	  readBlockSize=UE->frame_parms.samples_per_tti - UE->rx_offset_diff;
	  writeBlockSize=UE->frame_parms.samples_per_tti -UE->rx_offset_diff;
	}

	LOG_D(PHY,"reading rxp[0] %p (%p)\n",
	      rxp[0],UE->common_vars.rxdata[0]);
	AssertFatal(readBlockSize ==
		    UE->rfdevice.trx_read_func(&UE->rfdevice,
					       &timestamp,
					       rxp,
					       readBlockSize,
					       UE->frame_parms.nb_antennas_rx),"");
	LOG_D(PHY,"writing txp[0] %p (%p)\n",txp[0],UE->common_vars.txdata[0]);

	AssertFatal( writeBlockSize ==
		     UE->rfdevice.trx_write_func(&UE->rfdevice,
						 timestamp+
						 (2*UE->frame_parms.samples_per_tti) -
						 openair0_cfg[0].tx_sample_advance,
						 txp,
						 writeBlockSize,
						 UE->frame_parms.nb_antennas_tx,
						 1),"");
	
	if (UE->is_synchronized == 0) { 
	  // IF we have no non-SL communications, wakeup RX/TX processing, otherwise it is done by main UE_thread
	  pickTime(gotIQs);
	  // operate on thread sf mod 2
	  AssertFatal(pthread_mutex_lock(&proc->mutex_rxtx) ==0,"");
	  if(sub_frame == 0) {
	    //UE->proc.proc_rxtx[0].frame_rx++;
	    //UE->proc.proc_rxtx[1].frame_rx++;
	    for (th_id=0; th_id < RX_NB_TH; th_id++) {
	      UE->proc.proc_rxtx[th_id].frame_rx++;
	    }
	  }
	  //UE->proc.proc_rxtx[0].gotIQs=readTime(gotIQs);
	  //UE->proc.proc_rxtx[1].gotIQs=readTime(gotIQs);
	  for (th_id=0; th_id < RX_NB_TH; th_id++) {
	    UE->proc.proc_rxtx[th_id].gotIQs=readTime(gotIQs);
	  }
	  proc->subframe_rx=sub_frame;
	  proc->subframe_tx=(sub_frame+4)%10;
	  proc->frame_tx = (proc->frame_rx + (proc->subframe_rx>5?1:0))&1023;
	  proc->timestamp_tx = timestamp+(4*UE->frame_parms.samples_per_tti);
	  	  
	  proc->instance_cnt_rxtx++;
	  LOG_D( PHY, "[SCHED][UE %d] UE RX instance_cnt_rxtx %d subframe %d !!\n", UE->Mod_id, proc->instance_cnt_rxtx,proc->subframe_rx);
	  if (proc->instance_cnt_rxtx == 0) {
	    if (pthread_cond_signal(&proc->cond_rxtx) != 0) {
	      LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
	      exit_fun("nothing to add");
	    }
	  } else {
	    LOG_E( PHY, "[SCHED][UE %d] UE RX thread busy (IC %d)!!\n", UE->Mod_id, proc->instance_cnt_rxtx);
	    if (proc->instance_cnt_rxtx > 2)
	      exit_fun("instance_cnt_rxtx > 2");
	  }
	  
	  AssertFatal (pthread_cond_signal(&proc->cond_rxtx) ==0 ,"");
	  AssertFatal(pthread_mutex_unlock(&proc->mutex_rxtx) ==0,"");
	  initRefTimes(t1);
	  initStaticTime(lastTime);
	  updateTimes(lastTime, &t1, 20000, "Delay between two IQ acquisitions (case 1)");
	  pickStaticTime(lastTime);
	  
	} // UE->is_synchronized==0
      } // start_rx_stream==1
    }   //  UE->is_synchronized==0 && UE->is_SynchRef==0
  } // while !oai_exit

  for (int i=0; i<UE->frame_parms.nb_antennas_rx;i++) free(rxdata[i]);
  
  return NULL;
}

#ifdef OPENAIR2
void fill_ue_band_info(void) {

  UE_EUTRA_Capability_t *UE_EUTRA_Capability = UE_rrc_inst[0].UECap->UE_EUTRA_Capability;
  int i,j;

  bands_to_scan.nbands = UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.count;

  for (i=0; i<bands_to_scan.nbands; i++) {

    for (j=0; j<sizeof (eutra_bands) / sizeof (eutra_bands[0]); j++)
      if (eutra_bands[j].band == UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.array[i]->bandEUTRA) {
	memcpy(&bands_to_scan.band_info[i],
	       &eutra_bands[j],
	       sizeof(eutra_band_t));

	printf("Band %d (%lu) : DL %u..%u Hz, UL %u..%u Hz, Duplex %s \n",
	       bands_to_scan.band_info[i].band,
	       UE_EUTRA_Capability->rf_Parameters.supportedBandListEUTRA.list.array[i]->bandEUTRA,
	       bands_to_scan.band_info[i].dl_min,
	       bands_to_scan.band_info[i].dl_max,
	       bands_to_scan.band_info[i].ul_min,
	       bands_to_scan.band_info[i].ul_max,
	       (bands_to_scan.band_info[i].frame_type==FDD) ? "FDD" : "TDD");
	break;
      }
  }
}
#endif

int setup_ue_buffers(PHY_VARS_UE **phy_vars_ue, openair0_config_t *openair0_cfg) {

  int i, CC_id;
  LTE_DL_FRAME_PARMS *frame_parms;
  openair0_rf_map *rf_map;

  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    rf_map = &phy_vars_ue[CC_id]->rf_map;

    AssertFatal( phy_vars_ue[CC_id] !=0, "");
    frame_parms = &(phy_vars_ue[CC_id]->frame_parms);

    // replace RX signal buffers with mmaped HW versions
    rxdata = (int32_t**)malloc16( frame_parms->nb_antennas_rx*sizeof(int32_t*) );
    txdata = (int32_t**)malloc16( frame_parms->nb_antennas_tx*sizeof(int32_t*) );

    for (i=0; i<frame_parms->nb_antennas_rx; i++) {
      LOG_I(PHY, "Mapping UE CC_id %d, rx_ant %d, freq %u on card %d, chain %d\n",
	    CC_id, i, downlink_frequency[CC_id][i], rf_map->card, rf_map->chain+i );
      free( phy_vars_ue[CC_id]->common_vars.rxdata[i] );
      rxdata[i] = (int32_t*)malloc16_clear( 307200*sizeof(int32_t) );
      phy_vars_ue[CC_id]->common_vars.rxdata[i] = rxdata[i]; // what about the "-N_TA_offset" ? // N_TA offset for TDD
    }

    for (i=0; i<frame_parms->nb_antennas_tx; i++) {
      LOG_I(PHY, "Mapping UE CC_id %d, tx_ant %d, freq %u on card %d, chain %d\n",
	    CC_id, i, downlink_frequency[CC_id][i], rf_map->card, rf_map->chain+i );
      free( phy_vars_ue[CC_id]->common_vars.txdata[i] );
      txdata[i] = (int32_t*)malloc16_clear( 307200*sizeof(int32_t) );
      phy_vars_ue[CC_id]->common_vars.txdata[i] = txdata[i];
    }

    // rxdata[x] points now to the same memory region as phy_vars_ue[CC_id]->common_vars.rxdata[x]
    // txdata[x] points now to the same memory region as phy_vars_ue[CC_id]->common_vars.txdata[x]
    // be careful when releasing memory!
    // because no "release_ue_buffers"-function is available, at least rxdata and txdata memory will leak (only some bytes)
  }
  return 0;
}


// Panos: This timer thread is used only in the phy_sub mode as an independent timer
// which will be ticking and provide the SFN/SF values that will be used from the UE threads
// playing the role of nfapi-pnf.



/*static void* timer_thread( void* param ) {
  thread_top_init("timer_thread",1,870000L,1000000L,1000000L);
  timer_subframe =9;
  timer_frame    =1023;
  //phy_stub_ticking = (SF_ticking*)malloc(sizeof(SF_ticking));
  phy_stub_ticking->ticking_var = -1;
  PHY_VARS_UE *UE;
  UE = PHY_vars_UE_g[0][0];
  double t_diff;
  int external_timer = 0;


  //struct timespec pselect_start;


  //struct timespec sf_duration;
  //sf_duration.tv_sec = 0;
  //sf_duration.tv_nsec = 1e6;


  wait_sync("timer_thread");

  //pthread_mutex_init(&phy_stub_ticking->mutex_ticking,NULL);
  //pthread_cond_init(&phy_stub_ticking->cond_ticking,NULL);

  //  struct timespec start = {0};
  //  struct timespec end = {0};
  //sleepValue.tv_nsec = 1000000;
  opp_enabled = 1;

  // first check if we are receiving timing indications
  if(nfapi_mode==4) {
  usleep(10000);
  if (UE->instance_cnt_timer > 0) {
  external_timer = 1;
  int absSFm1 = ((emulator_absSF+10239)%10240);
  timer_frame = absSFm1/10;
  timer_subframe = absSFm1%10;
  pthread_mutex_lock(&UE->timer_mutex);
  UE->instance_cnt_timer = -1;
  pthread_mutex_unlock(&UE->timer_mutex);
  LOG_I(PHY,"Running with external timer\n");
  }
  else LOG_I(PHY,"Running with internal timer\n");
  }

  struct timespec t_start;
  struct timespec t_now;
  struct timespec t_sleep;
  uint64_t T_0;
  uint64_t T_now;
  uint64_t T_next_SF;
  uint64_t T_sleep;
  uint64_t sf_cnt = 0; //Total Subframe counter

  clock_gettime(CLOCK_MONOTONIC, &t_start);
  T_0 = (uint64_t) t_start.tv_sec*1000000000 + t_start.tv_nsec;
  LOG_I(MAC, "Panos-D: timer_thread(), T_0 value: %" PRId64 "\n", T_0);
  //printf("%" PRId64 "\n", t);

  while (!oai_exit) {

  // these are local subframe/frame counters to check that we are in synch with the fronthaul timing.
  // They are set on the first rx/tx in the underly FH routines.
  if (timer_subframe==9) {
  timer_subframe=0;
  timer_frame++;
  timer_frame&=1023;
  } else {
  timer_subframe++;
  }
  //printf("[timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
  //LOG_I(MAC," Panos-D [timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
  //AssertFatal( 0 == pthread_cond_signal(&phy_stub_ticking->cond_ticking), "");
  AssertFatal(pthread_mutex_lock(&phy_stub_ticking->mutex_ticking) ==0,"");
  phy_stub_ticking->ticking_var++;
  // This should probably be a call to pthread_cond_broadcast when we introduce support for multiple UEs (threads)
  if(phy_stub_ticking->ticking_var == 0){
  //AssertFatal(phy_stub_ticking->ticking_var == 0,"phy_stub_ticking->ticking_var = %d",
  //phy_stub_ticking->ticking_var);
  if (pthread_cond_signal(&phy_stub_ticking->cond_ticking) != 0) {
  //LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
  LOG_E( PHY, "timer_thread ERROR pthread_cond_signal for UE_thread\n");
  exit_fun("nothing to add");
  }
  }
  else
  LOG_I(MAC, "Panos-D: timer_thread() Timing problem! \n");

  AssertFatal(pthread_mutex_unlock(&phy_stub_ticking->mutex_ticking) ==0,"");
  start_meas(&UE->timer_stats);


  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start); // get initial time-stamp
  if (external_timer == 0) {
  sf_cnt++;
  T_next_SF = T_0 + sf_cnt*1000000;
  do{
  clock_gettime(CLOCK_MONOTONIC, &t_now);
  T_now =(uint64_t) t_now.tv_sec*1000000000 + t_now.tv_nsec;
  }while(T_now < T_next_SF);
  //usleep(1000);
  UE_tport_t pdu;
  pdu.header.packet_type = TTI_SYNC;
  pdu.header.absSF = (timer_frame*10)+timer_subframe;
  multicast_link_write_sock(0,
  &pdu,
  sizeof(UE_tport_header_t));

  }
  else {
  wait_on_condition(&UE->timer_mutex,&UE->timer_cond,&UE->instance_cnt_timer,"timer_thread");
  release_thread(&UE->timer_mutex,&UE->instance_cnt_timer,"timer_thread");
  }
  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);   // get final time-stamp

  //double t_ns = (double)(end.tv_sec - start.tv_sec) * 1.0e9 +
  //              (double)(end.tv_nsec - start.tv_nsec);
  //printf("Panos-D: [timer_thread] REAL TIME difference: %f", t_ns);


  stop_meas(&UE->timer_stats);
  t_diff = get_time_meas_us(&UE->timer_stats);

  //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
  //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
  //if (t_diff > 1100)


  //    LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
  //LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);

  //printf("Panos-D: Absolute time: %f", t_diff);


  stop_meas(&UE->timer_stats);
  t_diff = get_time_meas_us(&UE->timer_stats);
  //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
  //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
  //if (t_diff > 1100) LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
  //printf("Panos-D: Absolute time: %f", t_diff);

  //UE->proc.ticking_var++;
  // pthread_cond_signal() //Send signal to ue_thread()?
  // We also need to somehow pass the information of SFN/SF
  }
  free(phy_stub_ticking);
  pthread_cond_destroy(&phy_stub_ticking->cond_ticking);
  pthread_mutex_destroy(&phy_stub_ticking->mutex_ticking);
  return 0;

  }*/











//02/02/2018
static void* timer_thread( void* param ) {
  thread_top_init("timer_thread",1,870000L,1000000L,1000000L);
  timer_subframe =9;
  timer_frame    =1023;
  //phy_stub_ticking = (SF_ticking*)malloc(sizeof(SF_ticking));
  phy_stub_ticking->ticking_var = -1;
  PHY_VARS_UE *UE;
  UE = PHY_vars_UE_g[0][0];
  double t_diff;
  int external_timer = 0;


  //struct timespec pselect_start;


  //struct timespec sf_duration;
  //sf_duration.tv_sec = 0;
  //sf_duration.tv_nsec = 1e6;


  wait_sync("timer_thread");

  //pthread_mutex_init(&phy_stub_ticking->mutex_ticking,NULL);
  //pthread_cond_init(&phy_stub_ticking->cond_ticking,NULL);

  //  struct timespec start = {0};
  //  struct timespec end = {0};
  //sleepValue.tv_nsec = 1000000;
  opp_enabled = 1;

  // first check if we are receiving timing indications
  if(nfapi_mode==4) {
    usleep(10000);
    if (UE->instance_cnt_timer > 0) {
      external_timer = 1;
      int absSFm1 = ((emulator_absSF+10239)%10240);
      timer_frame = absSFm1/10;
      timer_subframe = absSFm1%10;
      pthread_mutex_lock(&UE->timer_mutex);
      UE->instance_cnt_timer = -1;
      pthread_mutex_unlock(&UE->timer_mutex);
      LOG_I(PHY,"Running with external timer\n");
    }
    else LOG_I(PHY,"Running with internal timer\n");
  }

  struct timespec t_start;
  struct timespec t_now;
  struct timespec t_sleep;
  uint64_t T_0;
  uint64_t T_now;
  uint64_t T_next_SF;
  uint64_t T_sleep;
  uint64_t sf_cnt = 0; //Total Subframe counter

  clock_gettime(CLOCK_MONOTONIC, &t_start);
  T_0 = (uint64_t) t_start.tv_sec*1000000000 + t_start.tv_nsec;
  LOG_I(MAC, "Panos-D: timer_thread(), T_0 value: %" PRId64 "\n", T_0);
  //printf("%" PRId64 "\n", t);

  while (!oai_exit) {

    // these are local subframe/frame counters to check that we are in synch with the fronthaul timing.
    // They are set on the first rx/tx in the underly FH routines.
    if (timer_subframe==9) {
      timer_subframe=0;
      timer_frame++;
      timer_frame&=1023;
    } else {
      timer_subframe++;
    }
    //printf("[timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
    //LOG_I(MAC," Panos-D [timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
    //AssertFatal( 0 == pthread_cond_signal(&phy_stub_ticking->cond_ticking), "");
    AssertFatal(pthread_mutex_lock(&phy_stub_ticking->mutex_ticking) ==0,"");
    phy_stub_ticking->ticking_var++;
    // This should probably be a call to pthread_cond_broadcast when we introduce support for multiple UEs (threads)
    if(phy_stub_ticking->ticking_var == 0){
      //AssertFatal(phy_stub_ticking->ticking_var == 0,"phy_stub_ticking->ticking_var = %d",
      //phy_stub_ticking->ticking_var);
      if (pthread_cond_signal(&phy_stub_ticking->cond_ticking) != 0) {
	//LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
	LOG_E( PHY, "timer_thread ERROR pthread_cond_signal for UE_thread\n");
	exit_fun("nothing to add");
      }
    }
    else
      LOG_I(MAC, "Panos-D: timer_thread() Timing problem! ticking_var value:%d \n \n \n", phy_stub_ticking->ticking_var);

    AssertFatal(pthread_mutex_unlock(&phy_stub_ticking->mutex_ticking) ==0,"");
    start_meas(&UE->timer_stats);


    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start); // get initial time-stamp
    if (external_timer == 0) {
      clock_gettime(CLOCK_MONOTONIC, &t_now);
      sf_cnt++;
      T_next_SF = T_0 + sf_cnt*1000000;
      T_now =(uint64_t) t_now.tv_sec*1000000000 + t_now.tv_nsec;
      if(T_now > T_next_SF){
	t_sleep.tv_sec =0;
	t_sleep.tv_nsec =0;
	//T_sleep=0;
      }
      else{
	T_sleep = T_next_SF - T_now;
	//LOG_I(MAC, "Panos-D: timer_thread(), T_sleep value: %" PRId64 "\n", T_sleep);
	t_sleep.tv_sec =0;
	t_sleep.tv_nsec = (__syscall_slong_t) T_sleep;
      }
      nanosleep(&t_sleep, (struct timespec *)NULL);
      //usleep(T_sleep/1000000);
      UE_tport_t pdu;
      pdu.header.packet_type = TTI_SYNC;
      pdu.header.absSF = (timer_frame*10)+timer_subframe;
      if (nfapi_mode!=3){
	multicast_link_write_sock(0,
				  &pdu,
				  sizeof(UE_tport_header_t));
      }

    }
    else {
      wait_on_condition(&UE->timer_mutex,&UE->timer_cond,&UE->instance_cnt_timer,"timer_thread");
      release_thread(&UE->timer_mutex,&UE->instance_cnt_timer,"timer_thread");
    }
    //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);   // get final time-stamp

    //double t_ns = (double)(end.tv_sec - start.tv_sec) * 1.0e9 +
    //              (double)(end.tv_nsec - start.tv_nsec);
    //printf("Panos-D: [timer_thread] REAL TIME difference: %f", t_ns);


    stop_meas(&UE->timer_stats);
    t_diff = get_time_meas_us(&UE->timer_stats);

    //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
    //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
    //if (t_diff > 1100)


    //    LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
    //LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);

    //printf("Panos-D: Absolute time: %f", t_diff);


    stop_meas(&UE->timer_stats);
    t_diff = get_time_meas_us(&UE->timer_stats);
    //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
    //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
    //if (t_diff > 1100) LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
    //printf("Panos-D: Absolute time: %f", t_diff);

    //UE->proc.ticking_var++;
    // pthread_cond_signal() //Send signal to ue_thread()?
    // We also need to somehow pass the information of SFN/SF
  }
  free(phy_stub_ticking);
  pthread_cond_destroy(&phy_stub_ticking->cond_ticking);
  pthread_mutex_destroy(&phy_stub_ticking->mutex_ticking);
  return 0;

}



















/*static void* timer_thread( void* param ) {
  thread_top_init("timer_thread",1,870000L,1000000L,1000000L);
  timer_subframe =9;
  timer_frame    =1023;
  //phy_stub_ticking = (SF_ticking*)malloc(sizeof(SF_ticking));
  phy_stub_ticking->ticking_var = -1;
  PHY_VARS_UE *UE;
  UE = PHY_vars_UE_g[0][0];
  double t_diff;
  int external_timer = 0;

  wait_sync("timer_thread");

  //pthread_mutex_init(&phy_stub_ticking->mutex_ticking,NULL);
  //pthread_cond_init(&phy_stub_ticking->cond_ticking,NULL);

  //  struct timespec start = {0};
  //  struct timespec end = {0};
  //sleepValue.tv_nsec = 1000000;
  opp_enabled = 1;

  // first check if we are receiving timing indications
  if(nfapi_mode==4) {
  usleep(10000);
  if (UE->instance_cnt_timer > 0) {
  external_timer = 1;
  int absSFm1 = ((emulator_absSF+10239)%10240);
  timer_frame = absSFm1/10;
  timer_subframe = absSFm1%10;
  pthread_mutex_lock(&UE->timer_mutex);
  UE->instance_cnt_timer = -1;
  pthread_mutex_unlock(&UE->timer_mutex);
  LOG_I(PHY,"Running with external timer\n");
  }
  else LOG_I(PHY,"Running with internal timer\n");
  }

  while (!oai_exit) {

  // these are local subframe/frame counters to check that we are in synch with the fronthaul timing.
  // They are set on the first rx/tx in the underly FH routines.
  if (timer_subframe==9) {
  timer_subframe=0;
  timer_frame++;
  timer_frame&=1023;
  } else {
  timer_subframe++;
  }
  //printf("[timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
  //LOG_I(MAC," Panos-D [timer_thread] Frame: %d, Subframe: %d \n", timer_frame, timer_subframe);
  //AssertFatal( 0 == pthread_cond_signal(&phy_stub_ticking->cond_ticking), "");
  AssertFatal(pthread_mutex_lock(&phy_stub_ticking->mutex_ticking) ==0,"");
  phy_stub_ticking->ticking_var++;
  // This should probably be a call to pthread_cond_broadcast when we introduce support for multiple UEs (threads)
  if(phy_stub_ticking->ticking_var == 0){
  //AssertFatal(phy_stub_ticking->ticking_var == 0,"phy_stub_ticking->ticking_var = %d",
  //		phy_stub_ticking->ticking_var);
  if (pthread_cond_signal(&phy_stub_ticking->cond_ticking) != 0) {
  //LOG_E( PHY, "[SCHED][UE %d] ERROR pthread_cond_signal for UE RX thread\n", UE->Mod_id);
  LOG_E( PHY, "timer_thread ERROR pthread_cond_signal for UE_thread\n");
  exit_fun("nothing to add");
  }
  }

  AssertFatal(pthread_mutex_unlock(&phy_stub_ticking->mutex_ticking) ==0,"");
  start_meas(&UE->timer_stats);


  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start); // get initial time-stamp
  if (external_timer == 0) {
  usleep(1000);
  UE_tport_t pdu;
  pdu.header.packet_type = TTI_SYNC;
  pdu.header.absSF = (timer_frame*10)+timer_subframe;
  multicast_link_write_sock(0,
  &pdu,
  sizeof(UE_tport_header_t));

  }
  else {
  wait_on_condition(&UE->timer_mutex,&UE->timer_cond,&UE->instance_cnt_timer,"timer_thread");
  release_thread(&UE->timer_mutex,&UE->instance_cnt_timer,"timer_thread");
  }
  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);   // get final time-stamp

  //double t_ns = (double)(end.tv_sec - start.tv_sec) * 1.0e9 +
  //              (double)(end.tv_nsec - start.tv_nsec);
  //printf("Panos-D: [timer_thread] REAL TIME difference: %f", t_ns);


  stop_meas(&UE->timer_stats);
  t_diff = get_time_meas_us(&UE->timer_stats);

  //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
  //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
  //if (t_diff > 1100)


  //    LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
  //LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);

  //printf("Panos-D: Absolute time: %f", t_diff);


  stop_meas(&UE->timer_stats);
  t_diff = get_time_meas_us(&UE->timer_stats);
  //printf("Panos-D: Absolute time: %lld, diff: %lld, diff_now: %lld \n",UE->timer_stats.p_time, UE->timer_stats.diff, UE->timer_stats.diff_now);
  //LOG_I(MAC,"[UE%d] Applying default macMainConfig\n",module_idP);
  //if (t_diff > 1100) LOG_E(MAC," Panos-D Absolute time: %f\n", t_diff);
  //printf("Panos-D: Absolute time: %f", t_diff);

  //UE->proc.ticking_var++;
  // pthread_cond_signal() //Send signal to ue_thread()?
  // We also need to somehow pass the information of SFN/SF
  }
  free(phy_stub_ticking);
  pthread_cond_destroy(&phy_stub_ticking->cond_ticking);
  pthread_mutex_destroy(&phy_stub_ticking->mutex_ticking);
  return 0;

  }*/








int init_timer_thread(void) {
  // Panos: CAREFUL Originally this was set to PHY_VARS_UE *UE=PHY_vars_UE_g[0]
  //PHY_VARS_UE *UE=PHY_vars_UE_g[0];
  PHY_VARS_UE *UE=PHY_vars_UE_g[0][0];
  phy_stub_ticking = (SF_ticking*)malloc(sizeof(SF_ticking));
  pthread_mutex_init(&UE->timer_mutex,NULL);
  pthread_cond_init(&UE->timer_cond,NULL);
  UE->instance_cnt_timer = -1;
  pthread_mutex_init(&phy_stub_ticking->mutex_ticking,NULL);
  pthread_cond_init(&phy_stub_ticking->cond_ticking,NULL);
  pthread_create(&phy_stub_ticking->pthread_timer, NULL, &timer_thread, NULL);
  return 0;
}
