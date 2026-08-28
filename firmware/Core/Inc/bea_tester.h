/*********************************************************/
/***** BEA Tester - cau noi UART (PC) <-> CAN1 (bus) ******/
/*********************************************************/
/*
 * Dong vai "Simulated ECU 1 SW (As Diagnostic Tool)" trong so do topology:
 * chuyen tiep payload UDS giua PC va bus CAN. Module nay KHONG hieu UDS -
 * no chi thao/boc vo khung. Moi logic chan doan nam o phia CAN2 (dcm*.c).
 *
 * Khung BEA tren duong UART:  0F FF F0 | payload | F0 00 0F
 */

#ifndef _BEA_TESTER_H
#define _BEA_TESTER_H

#include "main.h"

/* Dau/cuoi khung BEA - phai khop config/constants.py cua tool Python */
#define BEA_SOF_0        0x0Fu
#define BEA_SOF_1        0xFFu
#define BEA_SOF_2        0xF0u
#define BEA_EOF_0        0xF0u
#define BEA_EOF_1        0x00u
#define BEA_EOF_2        0x0Fu

#define BEA_MAX_PAYLOAD  64u

/* Dat 1 de in thong tin chan doan ra UART (dung khi go loi bus CAN).
 * Dat 0 khi da chay on de duong UART chi con khung BEA sach. */
#define BEA_DEBUG_TRACE  1

/* Bo dem so khung nhan duoc tren tung CAN - tang trong ISR, doc de go loi. */
extern volatile uint32_t g_Can1RxCount;
extern volatile uint32_t g_Can2RxCount;

/* Goi mot lan trong main(), sau khi cac ngoai vi da san sang. */
void BeaTester_Init(void);

/* Goi tu HAL_UART_RxCpltCallback - nap tung byte vao may trang thai do khung.
 * Chay trong ngat: chi do khung va chep bo dem, khong gui gi. */
void BeaTester_OnUartByte(uint8_t b);

/* Goi tu CAN1_RX0_IRQHandler khi nhan duoc dap ung tu ECU (ID 0x7A2).
 * Chay trong ngat: chi chep bo dem. */
void BeaTester_OnCanResponse(const uint8_t *data, uint8_t len);

/* Goi tu vong lap chinh - lam moi viec "nang": phat khung CAN va gui UART. */
void BeaTester_Periodic(void);

#endif
