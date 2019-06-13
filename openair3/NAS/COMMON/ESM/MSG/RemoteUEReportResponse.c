/*
 * RemoteUEReportResponse.c
 *
 *  Created on: Jun 7, 2019
 *      Author: nepes
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#include "TLVEncoder.h"
#include "TLVDecoder.h"
#include "RemoteUEReportResponse.h"

int decode_remote_ue_report_response(remote_ue_report_response_msg *remoteuereportresponse, uint8_t *buffer, uint32_t len)
{
  uint32_t decoded = 0;

  // Check if we got a NULL pointer and if buffer length is >= minimum length expected for the message.
  CHECK_PDU_POINTER_AND_LENGTH_DECODER(buffer, REMOTE_UE_REPORT_RESPONSE_MINIMUM_LENGTH, len);

  /* Decoding mandatory fields */
  return decoded;
}

int encode_remote_ue_report_response(remote_ue_report_response_msg *remoteuereportresponse, uint8_t *buffer, uint32_t len)
{
  int encoded = 0;

  /* Checking IEI and pointer */
  CHECK_PDU_POINTER_AND_LENGTH_ENCODER(buffer, REMOTE_UE_REPORT_RESPONSE_MINIMUM_LENGTH, len);

  return encoded;
}

