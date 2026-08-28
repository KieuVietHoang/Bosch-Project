/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_H
#define _DCM_H

#include "main.h"
#include "stm32f4xx_it.h"
#include "can_tp.h"

/* ---- Network layer (spec: Diagnostic CAN ID) ------------------------ */
#define DCM_REQ_CAN_ID_DEFAULT   0x712u   /* Tester -> ECU, changeable via $2E */
#define DCM_RESP_CAN_ID          0x7A2u   /* ECU -> Tester, fixed */

/* ---- Service Identifiers --------------------------------------------- */
#define SID_READ_DID         0x22u
#define SID_SECURITY         0x27u
#define SID_WRITE_DID        0x2Eu
#define SID_POSITIVE_OFFSET  0x40u

/* ---- Negative Response Codes ------------------------------------------*/
#define NRC_GENERAL_REJECT       0x10u
#define NRC_INVALID_LENGTH       0x13u
#define NRC_DID_NOT_SUPPORTED    0x31u
#define NRC_SECURITY_DENIED      0x33u
#define NRC_INVALID_KEY          0x35u

/* ---- Data Identifiers -------------------------------------------------*/
#define DID_TESTER_CANID   0x0123u   /* Practice 1 ($22) / Practice 4 ($2E) */
#define DID_TEMPERATURE    0x0124u   /* Practice 2 ($22) */

/* Dispatch one UDS request payload (SID + data, WITHOUT the CAN-TP PCI
 * byte) received on the currently-accepted request CAN ID. */
void Dcm_HandleRequest(const uint8_t *reqData, uint8_t reqLen);

/* Call once from main() after peripherals are up. */
void Dcm_Init(void);

/* Call periodically from the main loop (~ms resolution is enough) to run
 * SecurityAccess timeouts: 5 s unlock window, 10 s penalty. */
void Dcm_Periodic(void);

/* Call from the ignition-cycle branch in main() (BtnU). Applies a pending
 * CAN ID change requested by a previous $2E write, if any. */
void Dcm_ApplyPendingCanId(void);

/* Current request CAN ID the ECU listens on (starts at 0x712, changeable
 * via $2E DID 0x0123 + ignition cycle). Used by CAN2_RX0_IRQHandler to
 * accept/reject incoming frames, and by dcm_rdbi to answer DID 0x0123. */
uint32_t Dcm_GetCurrentReqCanId(void);

/* Called by dcm_wdbi after a successful write of DID 0x0123: stages a new
 * request CAN ID, applied later by Dcm_ApplyPendingCanId(). */
void Dcm_SetPendingCanId(uint32_t newCanId);

/* Shared response helpers used by all three service handlers. */
void Dcm_SendPositive(const uint8_t *data, uint8_t len);
void Dcm_SendNegative(uint8_t requestSid, uint8_t nrc);

#endif
