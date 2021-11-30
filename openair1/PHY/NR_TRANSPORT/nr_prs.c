
#include "PHY/defs_gNB.h"
#include "PHY/NR_TRANSPORT/nr_transport_proto.h"
#include "PHY/LTE_REFSIG/lte_refsig.h"
#include "PHY/NR_REFSIG/nr_refsig.h"
#include "PHY/sse_intrin.h"

//#define DEBUG_PRS
#define DEBUG_PRS_MAP

extern short nr_qpsk_mod_table[8];

int nr_generate_prs(uint32_t *nr_gold_prs,
                          int32_t *txdataF,
                          int16_t amp,
                          uint8_t ssb_start_symbol,
                          nfapi_nr_config_request_scf_t *config,
                          NR_DL_FRAME_PARMS *frame_parms) {
  
  
  // Get K_Prime from the table for the length of PRS or LPRS
  int k_prime_table[4][12] = {
        {0,1,0,1,0,1,0,1,0,1,0,1},
        {0,2,1,3,0,2,1,3,0,2,1,3},
        {0,3,1,4,2,5,0,3,1,4,2,5},
        {0,6,3,9,1,7,4,10,2,8,5,11}};
    
  int k_prime = 0;
  int k=0;
  int16_t mod_prs[NR_MAX_PRS_LENGTH<<1];
  uint8_t idx=0;
  uint8_t combSize = 6;
  uint8_t REOffset = 0;
  uint8_t symbolStart = 5;
  uint8_t NumPRSSymbols = 6;
  
  // QPSK modulation
  for (int m=0; m<NR_MAX_PRS_LENGTH; m++) {
    idx = (((nr_gold_prs[(m<<1)>>5])>>((m<<1)&0x1f))&3);
    mod_prs[m<<1] = nr_qpsk_mod_table[idx<<1];
    mod_prs[(m<<1)+1] = nr_qpsk_mod_table[(idx<<1) + 1];
    
#ifdef DEBUG_PRS
    printf("m %d idx %d gold seq %d b0-b1 %d-%d mod_prs %d %d\n", m, idx, nr_gold_prs[(m<<1)>>5], (((nr_gold_prs[(m<<1)>>5])>>((m<<1)&0x1f))&1),
           (((nr_gold_prs[((m<<1)+1)>>5])>>(((m<<1)+1)&0x1f))&1), mod_prs[(m<<1)], mod_prs[(m<<1)+1]);
#endif
  }
  
   // PRS resource mapping with combsize=k which means PRS symbols exist in every k-th subcarrier in frequency domain
   // According to ts138.211 sec.7.4.1.7.2

  for (int l = symbolStart; l < symbolStart + NumPRSSymbols; l++) {

  int symInd = l-5;
        if (combSize==2) {
            k_prime = k_prime_table[0][symInd];
        }
        else if (combSize==4){
            k_prime = k_prime_table[1][symInd];
        }
        else if (combSize==6){
            k_prime = k_prime_table[2][symInd];
        }
        else if (combSize==12){
            k_prime = k_prime_table[3][symInd];
        }

  for (int m = 0; m < NR_MAX_PRS_LENGTH; m++) {
#ifdef DEBUG_PRS_MAP
    printf("m %d at k %d of l %d\n", m, k, l);
#endif


    ((int16_t *)txdataF)[(l*frame_parms->ofdm_symbol_size + k)<<1]       = (amp * mod_prs[m<<1]) >> 15;
    ((int16_t *)txdataF)[((l*frame_parms->ofdm_symbol_size + k)<<1) + 1] = (amp * mod_prs[(m<<1) + 1]) >> 15;

#ifdef DEBUG_PRS_MAP
    printf("(%d,%d)\n",
           ((int16_t *)txdataF)[(l*frame_parms->ofdm_symbol_size + k)<<1],
           ((int16_t *)txdataF)[((l*frame_parms->ofdm_symbol_size + k)<<1)+1]);
#endif
    
    k=m*combSize+(REOffset+k_prime)%combSize;

    //if (k >= frame_parms->ofdm_symbol_size)
      //k-=frame_parms->ofdm_symbol_size;
      
      }
  }
  
  return 0;
}
 



