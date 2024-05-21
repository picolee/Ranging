/*
 * ranging_edma.h
 *
 *  Created on: May 13, 2024
 *      Author: LeeLemay
 */

#ifndef INC_RANGING_EDMA_H_
#define INC_RANGING_EDMA_H_


void Ranging_edmaInit(void);
void Ranging_edmaOpen(void);
void Ranging_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo);
void Ranging_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo);



#endif /* INC_RANGING_EDMA_H_ */
