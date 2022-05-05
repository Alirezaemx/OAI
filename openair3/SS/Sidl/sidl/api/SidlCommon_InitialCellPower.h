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

typedef int16_t AbsoluteCellPower_Type;

typedef uint8_t AttenuationValue_Type;

enum Attenuation_Type_Sel {
	Attenuation_Type_UNBOUND_VALUE = 0,
	Attenuation_Type_Value = 1,
	Attenuation_Type_Off = 2,
};

union Attenuation_Type_Value {
	AttenuationValue_Type Value;
	Null_Type Off;
};

struct Attenuation_Type {
	enum Attenuation_Type_Sel d;
	union Attenuation_Type_Value v;
};

typedef struct Attenuation_Type InitialAttenuation_Type;

struct InitialCellPower_Type {
	AbsoluteCellPower_Type MaxReferencePower;
	InitialAttenuation_Type Attenuation;
};

SIDL_END_C_INTERFACE
