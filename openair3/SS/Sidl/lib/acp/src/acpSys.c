/*
 *****************************************************************
 *
 * Module  : ACP - Asynchronous Communication Protocol Generated Services Code
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

#include "acpSys.h"
#include "acpCtx.h"
#include "acpProto.h"
#include "acpMsgIds.h"
#include "serSys.h"

int acpSysProcessEncClt(acpCtx_t _ctx, unsigned char* _buffer, size_t* _size, const struct SYSTEM_CTRL_REQ* FromSS)
{
	if (!acpCtxIsValid(_ctx)) {
		return -ACP_ERR_INVALID_CTX;
	}
	size_t _lidx = ACP_HEADER_SIZE;
	int _ret = serSysProcessEncClt(_buffer, *_size, &_lidx, FromSS);
	if (_ret == SIDL_STATUS_OK) {
		acpBuildHeader(_ctx, ACP_LID_SysProcess, _lidx, _buffer);
	}
	*_size = _lidx;
	return _ret;
}

int acpSysProcessDecSrv(acpCtx_t _ctx, const unsigned char* _buffer, size_t _size, struct SYSTEM_CTRL_REQ** FromSS)
{
	if (!acpCtxIsValid(_ctx)) {
		return -ACP_ERR_INVALID_CTX;
	}
	return serSysProcessDecSrv(_buffer + ACP_HEADER_SIZE, _size - ACP_HEADER_SIZE, ACP_CTX_CAST(_ctx)->arena, ACP_CTX_CAST(_ctx)->aSize, FromSS);
}

void acpSysProcessFreeSrv(struct SYSTEM_CTRL_REQ* FromSS)
{
	serSysProcessFreeSrv(FromSS);
}

int acpSysProcessEncSrv(acpCtx_t _ctx, unsigned char* _buffer, size_t* _size, const struct SYSTEM_CTRL_CNF* ToSS)
{
	if (!acpCtxIsValid(_ctx)) {
		return -ACP_ERR_INVALID_CTX;
	}
	size_t _lidx = ACP_HEADER_SIZE;
	int _ret = serSysProcessEncSrv(_buffer, *_size, &_lidx, ToSS);
	if (_ret == SIDL_STATUS_OK) {
		acpBuildHeader(_ctx, ACP_LID_SysProcess, _lidx, _buffer);
	}
	*_size = _lidx;
	return _ret;
}

int acpSysProcessDecClt(acpCtx_t _ctx, const unsigned char* _buffer, size_t _size, struct SYSTEM_CTRL_CNF** ToSS)
{
	if (!acpCtxIsValid(_ctx)) {
		return -ACP_ERR_INVALID_CTX;
	}
	return serSysProcessDecClt(_buffer + ACP_HEADER_SIZE, _size - ACP_HEADER_SIZE, ACP_CTX_CAST(_ctx)->arena, ACP_CTX_CAST(_ctx)->aSize, ToSS);
}

void acpSysProcessFreeClt(struct SYSTEM_CTRL_CNF* ToSS)
{
	serSysProcessFreeClt(ToSS);
}