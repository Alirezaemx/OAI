#ifndef COPY_FROM_INTERMEDIATE_REPRESENTATION_TO_ASN_MIR_H
#define COPY_FROM_INTERMEDIATE_REPRESENTATION_TO_ASN_MIR_H 

#include "f1ap_types/f1_setup.h"
#include "f1ap_types/f1_setup_response.h"
#include "f1ap_types/f1_setup_failure.h"
#include "f1ap_types/gnb_cu_conf_update.h"
#include "f1ap_types/gnb_cu_conf_update_ack.h"
#include "f1ap_types/ue_ctx_setup_request.h"
#include "f1ap_types/ue_ctx_setup_response.h"
#include "f1ap_types/init_ul_rrc_msg.h"
#include "f1ap_types/dl_rrc_msg.h"
#include "f1ap_types/ul_rrc_msg.h"
#include "f1ap_types/ue_ctx_mod_request.h"
#include "f1ap_types/ue_ctx_mod_resp.h"

#include "../../../cmake_targets/ran_build/build/CMakeFiles/F1AP_R16.3.1/F1AP_F1AP-PDU.h"

F1AP_F1AP_PDU_t cp_f1_setup_asn(f1_setup_t const* src);

F1AP_F1AP_PDU_t cp_f1_setup_response_asn(f1_setup_response_t const* src);

F1AP_F1AP_PDU_t cp_f1_setup_failure_asn(f1_setup_failure_t const* src);

F1AP_F1AP_PDU_t cp_ue_ctx_setup_request_asn(ue_ctx_setup_request_t const* src);

F1AP_F1AP_PDU_t cp_ue_ctx_setup_response_asn(ue_ctx_setup_response_t const* src);

F1AP_F1AP_PDU_t cp_gnb_cu_conf_update_asn(gnb_cu_conf_update_t const* src);

F1AP_F1AP_PDU_t cp_gnb_cu_conf_update_ack_asn( gnb_cu_conf_update_ack_t const* src);

F1AP_F1AP_PDU_t cp_init_ul_rrc_msg_asn(init_ul_rrc_msg_t const* src);

F1AP_F1AP_PDU_t cp_ul_rrc_msg_asn(ul_rrc_msg_t const* src);

F1AP_F1AP_PDU_t cp_dl_rrc_msg_asn(dl_rrc_msg_t const* src); 

F1AP_F1AP_PDU_t cp_ue_ctx_mod_req_asn(ue_ctx_mod_req_t const* src);

F1AP_F1AP_PDU_t cp_ue_ctx_mod_resp_asn(ue_ctx_mod_resp_t const* src);

#endif

