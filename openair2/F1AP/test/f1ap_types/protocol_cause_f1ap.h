#ifndef PROTOCOL_CAUSE_F1AP_MIR_H
#define PROTOCOL_CAUSE_F1AP_MIR_H 

typedef enum{

 TRANSFER_SYNTAX_ERROR_PROTOCOL_CAUSE_F1AP = 0,
 ABSTRACT_SYNTAX_ERROR_REJECT_PROTOCOL_CAUSE_F1AP = 1,
 ABSTRACT_SYNTAX_ERROR_IGNORE_AND_NOTIFY_PROTOCOL_CAUSE_F1AP = 2,
 MESSAGE_NOT_COMPATIBLE_WITH_RECEIVER_STATE_PROTOCOL_CAUSE_F1AP = 3,
 SEMANTIC_ERROR_PROTOCOL_CAUSE_F1AP = 4,
 ABSTRACT_SYNTAX_ERROR_FALSELY_CONSTRUCTED_MESSAGE_PROTOCOL_CAUSE_F1AP = 5,
 UNSPECIFIED_PROTOCOL_CAUSE_F1AP = 6,


  END_PROTOCOL_CAUSE_F1AP = 7,

} protocol_cause_f1ap_e ;



#endif

