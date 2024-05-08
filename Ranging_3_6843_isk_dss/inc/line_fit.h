/*
 * line_fit.h
 *
 *  Created on: May 4, 2024
 *              May the fourth be with you
 *      Author: LeeLemay
 */

#ifndef INC_LINE_FIT_H_
#define INC_LINE_FIT_H_


#include <stdint.h>



uint16_t line_fit(
    uint16_t n,
    uint32_t* x,
    uint32_t* y,
    float* m,
    float* b);


#endif /* INC_LINE_FIT_H_ */
