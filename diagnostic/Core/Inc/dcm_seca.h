/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_SECA_H
#define _DCM_SECA_H

#include "dcm.h"

#define DCM_SECA_LEVEL1_SEED_SUBFUNC   0x01u
#define DCM_SECA_LEVEL1_KEY_SUBFUNC    0x02u
#define DCM_SECA_UNLOCK_DURATION_MS    5000u
#define DCM_SECA_PENALTY_DURATION_MS   10000u

void    Dcm_Seca_Init(void);
void    Dcm_Seca_Periodic(void);
void    Dcm_Seca_Handle(const uint8_t *reqData, uint8_t reqLen);
uint8_t Dcm_Seca_IsLevel1Unlocked(void);

#endif
