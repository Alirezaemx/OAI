static inline void nrLDPC_bnProc_BG1_R89_AVX2(int8_t* bnProcBuf,int8_t* bnProcBufRes,  int8_t* llrRes, uint16_t Z ) {
        __m256i* p_bnProcBuf; 
        __m256i* p_bnProcBufRes; 
        __m256i* p_llrRes; 
        __m256i* p_res; 
        uint32_t M, i; 
// Process group with 2 CNs 
 M = (3*Z + 31)>>5;
    p_bnProcBuf     = (__m256i*) &bnProcBuf    [384];
   p_bnProcBufRes    = (__m256i*) &bnProcBufRes   [384];
            p_res = &p_bnProcBufRes[0];
            p_llrRes = (__m256i*) &llrRes  [384];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[0 + i]);
             p_res++;
             p_llrRes++;
}
            p_res = &p_bnProcBufRes[36];
            p_llrRes = (__m256i*) &llrRes  [384];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[36 + i]);
             p_res++;
             p_llrRes++;
}
// Process group with 3 CNs 
       M = (21*Z + 31)>>5;
    p_bnProcBuf     = (__m256i*) &bnProcBuf    [2688];
   p_bnProcBufRes    = (__m256i*) &bnProcBufRes   [2688];
            p_res = &p_bnProcBufRes[0];
            p_llrRes = (__m256i*) &llrRes  [1536];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[0 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[252];
            p_llrRes = (__m256i*) &llrRes  [1536];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[252 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[504];
            p_llrRes = (__m256i*) &llrRes  [1536];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[504 + i]);
            p_res++;
            p_llrRes++;
}
// Process group with 4 CNs 
       M = (1*Z + 31)>>5;
    p_bnProcBuf     = (__m256i*) &bnProcBuf    [26880];
   p_bnProcBufRes    = (__m256i*) &bnProcBufRes   [26880];
            p_res = &p_bnProcBufRes[0];
            p_llrRes = (__m256i*) &llrRes  [9600];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[0 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[12];
            p_llrRes = (__m256i*) &llrRes  [9600];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[12 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[24];
            p_llrRes = (__m256i*) &llrRes  [9600];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[24 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[36];
            p_llrRes = (__m256i*) &llrRes  [9600];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[36 + i]);
            p_res++;
            p_llrRes++;
}
// Process group with 5 CNs 
       M = (1*Z + 31)>>5;
    p_bnProcBuf     = (__m256i*) &bnProcBuf    [28416];
   p_bnProcBufRes    = (__m256i*) &bnProcBufRes   [28416];
            p_res = &p_bnProcBufRes[0];
            p_llrRes = (__m256i*) &llrRes  [9984];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[0 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[12];
            p_llrRes = (__m256i*) &llrRes  [9984];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[12 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[24];
            p_llrRes = (__m256i*) &llrRes  [9984];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[24 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[36];
            p_llrRes = (__m256i*) &llrRes  [9984];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[36 + i]);
            p_res++;
            p_llrRes++;
}
            p_res = &p_bnProcBufRes[48];
            p_llrRes = (__m256i*) &llrRes  [9984];
            for (i=0;i<M;i++) {
            *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[48 + i]);
            p_res++;
            p_llrRes++;
}
// Process group with 6 CNs 
// Process group with 7 CNs 
// Process group with 8 CNs 
// Process group with 9 CNs 
// Process group with 10 CNs 
// Process group with 11 CNs 
// Process group with 12 CNs 
// Process group with 13 CNs 
// Process group with 14 CNs 
// Process group with 15 CNs 
// Process group with 16 CNs 
// Process group with 17 CNs 
// Process group with 18 CNs 
// Process group with 19 CNs 
// Process group with 20 CNs 
// Process group with 21 CNs 
// Process group with 22 CNs 
// Process group with <23 CNs 
// Process group with 24 CNs 
// Process group with 25 CNs 
// Process group with 26 CNs 
// Process group with 27 CNs 
// Process group with 28 CNs 
// Process group with 29 CNs 
// Process group with 30 CNs 
}
