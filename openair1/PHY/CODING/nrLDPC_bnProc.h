/*==============================================================================
* nrLDPC_bnProc.h
*
* Defines the functions for bit node processing
*
* Author: Sebastian Wagner
* Date: 17-11-2017
*
===============================================================================*/

#ifndef __NR_LDPC_BNPROC__H__
#define __NR_LDPC_BNPROC__H__

static inline void nrLDPC_bnProcPc(t_nrLDPC_lut* p_lut, uint16_t Z)
{
    const uint8_t*  lut_numBnInBnGroups = p_lut->numBnInBnGroups;
    const uint32_t* lut_startAddrBnGroups = p_lut->startAddrBnGroups;
    const uint16_t* lut_startAddrBnGroupsLlr = p_lut->startAddrBnGroupsLlr;

    __m128i* p_bnProcBuf;
    __m256i* p_bnProcBufRes;
    __m128i* p_llrProcBuf;
    __m256i* p_llrProcBuf256;
    __m256i* p_llrRes;

    // Number of BNs in Groups
    uint32_t M;
    //uint32_t M32rem;
    uint32_t i,j;
    uint32_t k;
    // Offset to each bit within a group in terms of 32 Byte
    uint32_t cnOffsetInGroup;
    uint8_t idxBnGroup = 0;

    __m256i ymm0, ymm1, ymmRes0, ymmRes1;

    // =====================================================================
    // Process group with 1 CN

    // There is always a BN group with 1 CN
    // Number of groups of 32 BNs for parallel processing
    M = (lut_numBnInBnGroups[0]*Z + 31)>>5;

    p_bnProcBuf     = (__m128i*) &bnProcBuf    [lut_startAddrBnGroups   [idxBnGroup]];
    p_bnProcBufRes  = (__m256i*) &bnProcBufRes [lut_startAddrBnGroups   [idxBnGroup]];
    p_llrProcBuf    = (__m128i*) &llrProcBuf   [lut_startAddrBnGroupsLlr[idxBnGroup]];
    p_llrProcBuf256 = (__m256i*) &llrProcBuf   [lut_startAddrBnGroupsLlr[idxBnGroup]];
    p_llrRes        = (__m256i*) &llrRes       [lut_startAddrBnGroupsLlr[idxBnGroup]];

    // Loop over BNs
    for (i=0,j=0; i<M; i++,j+=2)
    {
        // Store results in bnProcBufRes of first CN for further processing for next iteration
        // In case parity check fails
        p_bnProcBufRes[i] = p_llrProcBuf256[i];

        // First 16 LLRs of first CN
        ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf [j]);
        ymm1 = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);

        ymmRes0 = _mm256_adds_epi16(ymm0, ymm1);

        // Second 16 LLRs of first CN
        ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf [j+1]);
        ymm1 = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);

        ymmRes1 = _mm256_adds_epi16(ymm0, ymm1);

        // Pack results back to epi8
        ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
        // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
        // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
        *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

        // Next result
        p_llrRes++;
    }

    // =====================================================================
    // Process group with 2 CNs

    if (lut_numBnInBnGroups[1] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[1]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[1]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 2
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<2; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 3 CNs

    if (lut_numBnInBnGroups[2] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[2]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[2]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 3
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<3; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 4 CNs

    if (lut_numBnInBnGroups[3] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[3]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[3]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 4
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<4; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 5 CNs

    if (lut_numBnInBnGroups[4] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[4]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[4]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 5
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<5; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 6 CNs

    if (lut_numBnInBnGroups[5] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[5]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[5]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 6
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<6; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 7 CNs

    if (lut_numBnInBnGroups[6] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[6]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[6]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 7
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<7; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 8 CNs

    if (lut_numBnInBnGroups[7] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[7]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[7]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 8
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<8; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 9 CNs

    if (lut_numBnInBnGroups[8] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[8]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[8]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 9
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<9; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 10 CNs

    if (lut_numBnInBnGroups[9] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[9]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[9]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 10
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<10; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 11 CNs

    if (lut_numBnInBnGroups[10] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[10]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[10]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 11
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<11; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 12 CNs

    if (lut_numBnInBnGroups[11] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[11]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[11]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 12
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<12; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 13 CNs

    if (lut_numBnInBnGroups[12] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[12]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[12]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 13
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<13; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 14 CNs

    if (lut_numBnInBnGroups[13] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[13]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[13]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 14
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<14; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 15 CNs

    if (lut_numBnInBnGroups[14] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[14]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[14]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 15
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<15; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 16 CNs

    if (lut_numBnInBnGroups[15] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[15]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[15]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 16
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<16; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 17 CNs

    if (lut_numBnInBnGroups[16] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[16]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[16]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 16
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<17; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 18 CNs

    if (lut_numBnInBnGroups[17] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[17]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[17]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 18
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<18; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 19 CNs

    if (lut_numBnInBnGroups[18] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[18]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[18]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 19
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<19; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 20 CNs

    if (lut_numBnInBnGroups[19] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[19]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[19]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 20
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<20; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 21 CNs

    if (lut_numBnInBnGroups[20] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[20]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[20]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 21
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<21; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 22 CNs

    if (lut_numBnInBnGroups[21] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[21]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[21]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 22
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<22; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 23 CNs

    if (lut_numBnInBnGroups[22] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[22]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[22]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 23
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<23; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 24 CNs

    if (lut_numBnInBnGroups[23] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[23]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[23]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 24
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<24; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 25 CNs

    if (lut_numBnInBnGroups[24] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[24]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[24]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 25
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<25; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 26 CNs

    if (lut_numBnInBnGroups[25] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[25]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[25]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 26
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<26; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 27 CNs

    if (lut_numBnInBnGroups[26] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[26]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[26]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 27
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<27; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 28 CNs

    if (lut_numBnInBnGroups[27] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[27]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[27]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 28
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<28; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 29 CNs

    if (lut_numBnInBnGroups[28] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[28]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[28]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 29
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<29; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

    // =====================================================================
    // Process group with 30 CNs

    if (lut_numBnInBnGroups[29] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[29]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 16 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[29]*NR_LDPC_ZMAX)>>4;

        // Set pointers to start of group 30
        p_bnProcBuf  = (__m128i*) &bnProcBuf  [lut_startAddrBnGroups   [idxBnGroup]];
        p_llrProcBuf = (__m128i*) &llrProcBuf [lut_startAddrBnGroupsLlr[idxBnGroup]];
        p_llrRes     = (__m256i*) &llrRes     [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
        for (i=0,j=0; i<M; i++,j+=2)
        {
            // First 16 LLRs of first CN
            ymmRes0 = _mm256_cvtepi8_epi16(p_bnProcBuf[j]);
            ymmRes1 = _mm256_cvtepi8_epi16(p_bnProcBuf[j+1]);

            // Loop over CNs
            for (k=1; k<30; k++)
            {
                ymm0 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j]);
                ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

                ymm1 = _mm256_cvtepi8_epi16(p_bnProcBuf[k*cnOffsetInGroup + j+1]);
                ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);
            }

            // Add LLR from receiver input
            ymm0    = _mm256_cvtepi8_epi16(p_llrProcBuf[j]);
            ymmRes0 = _mm256_adds_epi16(ymmRes0, ymm0);

            ymm1    = _mm256_cvtepi8_epi16(p_llrProcBuf[j+1]);
            ymmRes1 = _mm256_adds_epi16(ymmRes1, ymm1);

            // Pack results back to epi8
            ymm0 = _mm256_packs_epi16(ymmRes0, ymmRes1);
            // ymm0     = [ymmRes1[255:128] ymmRes0[255:128] ymmRes1[127:0] ymmRes0[127:0]]
            // p_llrRes = [ymmRes1[255:128] ymmRes1[127:0] ymmRes0[255:128] ymmRes0[127:0]]
            *p_llrRes = _mm256_permute4x64_epi64(ymm0, 0xD8);

            // Next result
            p_llrRes++;
        }
    }

}

static inline void nrLDPC_bnProc(t_nrLDPC_lut* p_lut, uint16_t Z)
{
    // BN Processing calculating the values to send back to the CNs for next iteration
    // bnProcBufRes contains the sum of all edges to each BN at the start of each group

    const uint8_t*  lut_numBnInBnGroups = p_lut->numBnInBnGroups;
    const uint32_t* lut_startAddrBnGroups = p_lut->startAddrBnGroups;
    const uint16_t* lut_startAddrBnGroupsLlr = p_lut->startAddrBnGroupsLlr;

    __m256i* p_bnProcBuf;
    __m256i* p_bnProcBufRes;
    __m256i* p_llrRes;
    __m256i* p_res;

    // Number of BNs in Groups
    uint32_t M;
    //uint32_t M32rem;
    uint32_t i;
    uint32_t k;
    // Offset to each bit within a group in terms of 32 Byte
    uint32_t cnOffsetInGroup;
    uint8_t idxBnGroup = 0;

    // =====================================================================
    // Process group with 1 CN
    // Already done in bnProcBufPc

    // =====================================================================
    // Process group with 2 CNs

    if (lut_numBnInBnGroups[1] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[1]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[1]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 2
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<2; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes[lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 3 CNs

    if (lut_numBnInBnGroups[2] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[2]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[2]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 3
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<3; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes[lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 4 CNs

    if (lut_numBnInBnGroups[3] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[3]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[3]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 4
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<4; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes[lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 5 CNs

    if (lut_numBnInBnGroups[4] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[4]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[4]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 5
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<5; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 6 CNs

    if (lut_numBnInBnGroups[5] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[5]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[5]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 6
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<6; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes[lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 7 CNs

    if (lut_numBnInBnGroups[6] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[6]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[6]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 7
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<7; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 8 CNs

    if (lut_numBnInBnGroups[7] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[7]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[7]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 8
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<8; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 9 CNs

    if (lut_numBnInBnGroups[8] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[8]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[8]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 9
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<9; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 10 CNs

    if (lut_numBnInBnGroups[9] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[9]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[9]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 10
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<10; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 11 CNs

    if (lut_numBnInBnGroups[10] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[10]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[10]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 10
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<11; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 12 CNs

    if (lut_numBnInBnGroups[11] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[11]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[11]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 12
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<12; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

        // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 13 CNs

    if (lut_numBnInBnGroups[12] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[12]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[12]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 13
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<13; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 14 CNs

    if (lut_numBnInBnGroups[13] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[13]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[13]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 14
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<14; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 15 CNs

    if (lut_numBnInBnGroups[14] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[14]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[14]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 15
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<15; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 16 CNs

    if (lut_numBnInBnGroups[15] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[15]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[15]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 16
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<16; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 17 CNs

    if (lut_numBnInBnGroups[16] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[16]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[16]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 17
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<17; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 18 CNs

    if (lut_numBnInBnGroups[17] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[17]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[17]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 18
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<18; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 19 CNs

    if (lut_numBnInBnGroups[18] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[18]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[18]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 19
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<19; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 20 CNs

    if (lut_numBnInBnGroups[19] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[19]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[19]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 20
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<20; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 21 CNs

    if (lut_numBnInBnGroups[20] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[20]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[20]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 21
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<21; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 22 CNs

    if (lut_numBnInBnGroups[21] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[21]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[21]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 22
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<22; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 23 CNs

    if (lut_numBnInBnGroups[22] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[22]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[22]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 23
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<23; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 24 CNs

    if (lut_numBnInBnGroups[23] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[23]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[23]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 24
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<24; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 25 CNs

    if (lut_numBnInBnGroups[24] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[24]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[24]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 25
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<25; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 26 CNs

    if (lut_numBnInBnGroups[25] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[25]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[25]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 26
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<26; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 27 CNs

    if (lut_numBnInBnGroups[26] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[26]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[26]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 27
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<27; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 28 CNs

    if (lut_numBnInBnGroups[27] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[27]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[27]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 28
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<28; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 29 CNs

    if (lut_numBnInBnGroups[28] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[28]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[28]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 29
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<29; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

    // =====================================================================
    // Process group with 30 CNs

    if (lut_numBnInBnGroups[29] > 0)
    {
        // If elements in group move to next address
        idxBnGroup++;

        // Number of groups of 32 BNs for parallel processing
        M = (lut_numBnInBnGroups[29]*Z + 31)>>5;

        // Set the offset to each CN within a group in terms of 32 Byte
        cnOffsetInGroup = (lut_numBnInBnGroups[29]*NR_LDPC_ZMAX)>>5;

        // Set pointers to start of group 30
        p_bnProcBuf    = (__m256i*) &bnProcBuf   [lut_startAddrBnGroups[idxBnGroup]];
        p_bnProcBufRes = (__m256i*) &bnProcBufRes[lut_startAddrBnGroups[idxBnGroup]];

        // Loop over CNs
        for (k=0; k<30; k++)
        {
            p_res = &p_bnProcBufRes[k*cnOffsetInGroup];
            p_llrRes = (__m256i*) &llrRes [lut_startAddrBnGroupsLlr[idxBnGroup]];

            // Loop over BNs
            for (i=0; i<M; i++)
            {
                *p_res = _mm256_subs_epi8(*p_llrRes, p_bnProcBuf[k*cnOffsetInGroup + i]);

                p_res++;
                p_llrRes++;
            }
        }
    }

}

static inline void nrLDPC_llr2bit(int8_t* out, int8_t* llrOut, uint16_t numLLR)
{
    __m256i* p_llrOut = (__m256i*) llrOut;
    __m256i* p_out    = (__m256i*) out;
    int8_t* p_llrOut8;
    int8_t* p_out8;
    uint32_t i;
    uint32_t M  = numLLR>>5;
    uint32_t Mr = numLLR&31;

    const __m256i* p_zeros = (__m256i*) zeros256_epi8;
    const __m256i* p_ones  = (__m256i*) ones256_epi8;

    for (i=0; i<M; i++)
    {
        *p_out++ = _mm256_and_si256(*p_ones, _mm256_cmpgt_epi8(*p_zeros, *p_llrOut));
        p_llrOut++;
    }

    if (Mr > 0)
    {
        // Remaining LLRs that do not fit in multiples of 32 bytes
        p_llrOut8 = (int8_t*) p_llrOut;
        p_out8    = (int8_t*) p_out;

        for (i=0; i<Mr; i++)
        {
            if (p_llrOut8[i] < 0)
            {
                p_out8[i] = 1;
            }
            else
            {
                p_out8[i] = 0;
            }
        }
    }
}

static inline void nrLDPC_llr2bitPacked(int8_t* out, int8_t* llrOut, uint16_t numLLR)
{
    __m256i*  p_llrOut = (__m256i*)  llrOut;
    uint32_t* p_bits   = (uint32_t*) out;
    int8_t* p_llrOut8;
    uint32_t bitsTmp = 0;
    uint32_t i;
    uint32_t M  = numLLR>>5;
    uint32_t Mr = numLLR&31;

    for (i=0; i<M; i++)
    {
        *p_bits++ = _mm256_movemask_epi8(*p_llrOut);
        p_llrOut++;
    }

    if (Mr > 0)
    {
        // Remaining LLRs that do not fit in multiples of 32 bytes
        p_llrOut8 = (int8_t*) p_llrOut;

        for (i=0; i<Mr; i++)
        {
            if (p_llrOut8[i] < 0)
            {
                bitsTmp |= (1<<i);
            }
            else
            {
                bitsTmp &= (0<<i);
            }
        }
    }
    *p_bits = bitsTmp;
}

/*
static inline void nrLDPC_bnProcPc_test(t_nrLDPC_lut* p_lut, int8_t* llrIn, int8_t* llrOut, uint16_t numLLR, uint16_t Z, uint8_t BG)
{
    // Sum all edges and store in llrRes

    const uint32_t* lut_llr2CnProcBuf = p_lut->llr2CnProcBuf;
    const uint8_t* lut_numEdgesPerBn = p_lut->numEdgesPerBn;
    uint32_t idxLut = 0;
    uint32_t idxCnProcBuf = 0;
    uint8_t numEdges;
    uint32_t i;
    uint32_t j;
    uint32_t k;
    uint8_t startColParity = NR_LDPC_START_COL_PARITY_BG1;
    uint32_t colG1;
    uint16_t bnSum[16] __attribute__ ((aligned(32))) = {0};
    uint16_t llrTmp[16] __attribute__ ((aligned(32))) = {0};
    __m256i* p_bnSum256 = (__m256i*) &bnSum[0];
    __m256i* p_llrOut256 = (__m256i*) &llrTmp[0];

    if (BG == 2)
    {
        startColParity = NR_LDPC_START_COL_PARITY_BG2;
    }
    colG1 = startColParity*Z;

    // BNs connected to more than 1 CN
    for (k=0; k<startColParity; k++)
    {
        numEdges = lut_numEdgesPerBn[k];

        for (i=0; i<Z; i++)
        {
            bnSum[0] = 0;

            // Sum all edges
            for (j=0; j<numEdges; j++)
            {
                idxCnProcBuf = lut_llr2CnProcBuf[idxLut++];

                bnSum[0] += cnProcBufRes[idxCnProcBuf];
            }

            // Add LLR from receiver input
            bnSum[0] += llrIn[k*Z + i];
            // Signed saturation
            *p_llrOut256 = _mm256_packs_epi16(*p_bnSum256, *p_llrOut256);
            llrOut[k*Z + i] = llrTmp[0];
        }
    }

    // BNs connected to 1 CN
    for (i=colG1; i<numLLR; i++)
    {
        idxCnProcBuf = lut_llr2CnProcBuf[idxLut++];
        bnSum[0] = cnProcBufRes[idxCnProcBuf];

        cnProcBuf[idxCnProcBuf] = llrIn[i];

        // Add LLR from receiver input
        bnSum[0] += llrIn[i];
        // Signed saturation
        *p_llrOut256 = _mm256_packs_epi16(*p_bnSum256, *p_llrOut256);
        llrOut[i] = llrTmp[0];
    }
}
*/
/*
static inline void nrLDPC_bnProc_test(t_nrLDPC_lut* p_lut, int8_t* llrOut, uint16_t numLLR, uint16_t Z, uint8_t BG)
{
    // Sum all edges and store in llrRes

    const uint32_t* lut_llr2CnProcBuf = p_lut->llr2CnProcBuf;
    const uint8_t* lut_numEdgesPerBn = p_lut->numEdgesPerBn;
    uint32_t idxLut = 0;
    uint32_t idxCnProcBuf = 0;
    uint8_t numEdges;
    uint32_t i;
    uint32_t j;
    uint32_t k;
    uint8_t startColParity = NR_LDPC_START_COL_PARITY_BG1;
    uint32_t colG1;

    if (BG == 2)
    {
        startColParity = NR_LDPC_START_COL_PARITY_BG2;
    }
    colG1 = startColParity*Z;

    // BNs connected to more than 1 CN
    for (k=0; k<startColParity; k++)
    {
        numEdges = lut_numEdgesPerBn[k];

        for (i=0; i<Z; i++)
        {

            for (j=0; j<numEdges; j++)
            {
                idxCnProcBuf = lut_llr2CnProcBuf[idxLut++];

                cnProcBuf[idxCnProcBuf] = llrOut[k*Z + i] - cnProcBufRes[idxCnProcBuf];
            }
        }
    }

}
*/

#endif
