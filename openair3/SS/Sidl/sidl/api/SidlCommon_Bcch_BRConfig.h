/*
 *****************************************************************
 *
 * Module  : SIDL - structures definitions
 *
 * Purpose : THIS FILE IS AUTOMATICALLY GENERATED !
 *
 *****************************************************************
 *
 *  Copyright (c) 2014-2021 SEQUANS Communications.
 *  All rights reserved.
 *
 *  This is confidential and proprietary source code of SEQUANS
 *  Communications. The use of the present source code and all
 *  its derived forms is exclusively governed by the restricted
 *  terms and conditions set forth in the SEQUANS
 *  Communications' EARLY ADOPTER AGREEMENT and/or LICENCE
 *  AGREEMENT. The present source code and all its derived
 *  forms can ONLY and EXCLUSIVELY be used with SEQUANS
 *  Communications' products. The distribution/sale of the
 *  present source code and all its derived forms is EXCLUSIVELY
 *  RESERVED to regular LICENCE holder and otherwise STRICTLY
 *  PROHIBITED.
 *
 *****************************************************************
 */

#pragma once

#include "SidlCompiler.h"
#include "SidlASN1.h"
#include "SidlASN1Base.h"
#include "SidlBase.h"
#include "SidlCommon.h"
#include "SidlCommonBase.h"

SIDL_BEGIN_C_INTERFACE

typedef MasterInformationBlock_schedulingInfoSIB1_BR_r13 SchedulingInfoSIB1_BR_r13_Type;

typedef struct SystemInformationBlockType1_v1310_IEs_bandwidthReducedAccessRelatedInfo_r13 BandwidthReducedAccessRelatedInfo_Type;

struct int32_t_SI_SubframeOffsetList_Type_Dynamic {
	size_t d;
	int32_t* v;
};

typedef struct int32_t_SI_SubframeOffsetList_Type_Dynamic SI_SubframeOffsetList_Type;

struct Bcch_BRToPbchConfig_Type {
	bool EnableMIB_Repetition;
};

struct Sib1_BRSchedul_Type {
	SchedulingInfoSIB1_BR_r13_Type SchedulingInfoSIB1_BR_r13;
};

struct Sib1_BRSchedul_Type_Sib1_BRSchedul_Optional {
	bool d;
	struct Sib1_BRSchedul_Type v;
};

struct BandwidthReducedAccessRelatedInfo_Type_SiSchedul_Optional {
	bool d;
	BandwidthReducedAccessRelatedInfo_Type v;
};

struct SI_SubframeOffsetList_Type_SubframeOffsetList_Optional {
	bool d;
	SI_SubframeOffsetList_Type v;
};

struct Bcch_BRToPdschConfig_Type {
	struct Sib1_BRSchedul_Type_Sib1_BRSchedul_Optional Sib1_BRSchedul;
	struct BandwidthReducedAccessRelatedInfo_Type_SiSchedul_Optional SiSchedul;
	struct SI_SubframeOffsetList_Type_SubframeOffsetList_Optional SubframeOffsetList;
};

struct BCCH_DL_SCH_Message_BR_BR_SI_List_Type_Dynamic {
	size_t d;
	struct BCCH_DL_SCH_Message_BR* v;
};

typedef struct BCCH_DL_SCH_Message_BR_BR_SI_List_Type_Dynamic BR_SI_List_Type;

struct BR_SI_List_Type_BR_SegmentedSI_List_Type_Dynamic {
	size_t d;
	BR_SI_List_Type* v;
};

typedef struct BR_SI_List_Type_BR_SegmentedSI_List_Type_Dynamic BR_SegmentedSI_List_Type;

struct BCCH_BCH_Message_Bcch_BRInfo_Type_MIB_Optional {
	bool d;
	struct BCCH_BCH_Message v;
};

struct BCCH_DL_SCH_Message_BR_Bcch_BRInfo_Type_SIB1_Optional {
	bool d;
	struct BCCH_DL_SCH_Message_BR v;
};

struct BR_SI_List_Type_SIs_Optional {
	bool d;
	BR_SI_List_Type v;
};

struct BR_SegmentedSI_List_Type_SegmentedSIs_Optional {
	bool d;
	BR_SegmentedSI_List_Type v;
};

struct Bcch_BRInfo_Type {
	struct BCCH_BCH_Message_Bcch_BRInfo_Type_MIB_Optional MIB;
	struct BCCH_DL_SCH_Message_BR_Bcch_BRInfo_Type_SIB1_Optional SIB1;
	struct BR_SI_List_Type_SIs_Optional SIs;
	struct BR_SegmentedSI_List_Type_SegmentedSIs_Optional SegmentedSIs;
};

struct Bcch_BRToPbchConfig_Type_Pbch_Optional {
	bool d;
	struct Bcch_BRToPbchConfig_Type v;
};

struct Bcch_BRToPdschConfig_Type_Pdsch_Optional {
	bool d;
	struct Bcch_BRToPdschConfig_Type v;
};

struct Bcch_BRInfo_Type_BcchInfo_Optional {
	bool d;
	struct Bcch_BRInfo_Type v;
};

struct Null_Type_Bcch_BRConfig_Type_StopSib1Transmission_Optional {
	bool d;
	Null_Type v;
};

struct Bcch_BRConfig_Type {
	struct Bcch_BRToPbchConfig_Type_Pbch_Optional Pbch;
	struct Bcch_BRToPdschConfig_Type_Pdsch_Optional Pdsch;
	struct Bcch_BRInfo_Type_BcchInfo_Optional BcchInfo;
	struct Null_Type_Bcch_BRConfig_Type_StopSib1Transmission_Optional StopSib1Transmission;
};

SIDL_END_C_INTERFACE
