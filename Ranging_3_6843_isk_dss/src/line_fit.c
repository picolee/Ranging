/*
 * line_fit.c
 *
 *  Created on: May 4, 2024
 *      Author: LeeLemay
 */




#include "inc/line_fit.h"
#include <stdlib.h>
#include <math.h>                           /* math functions */


// Compute the coefficients of a line given noisy data
// n = number of data points
// x,y  = arrays of data
// *b = output intercept
// *m  = output slope
uint16_t line_fit(uint16_t n, uint32_t* x, uint32_t* y, float* m, float* b)
{
    float   sumx = 0.0;                      /* sum of x     */
    float   sumx2 = 0.0;                     /* sum of x**2  */
    float   sumxy = 0.0;                     /* sum of x * y */
    float   sumy = 0.0;                      /* sum of y     */
    float   sumy2 = 0.0;                     /* sum of y**2  */
    uint16_t index;

    for (index=0; index < n;index++)
    {
        sumx  += x[index];
        sumx2 += x[index] * x[index];
        sumxy += x[index] * y[index];
        sumy  += y[index];
        sumy2 += y[index] * y[index];
    }

    float denom = (n * sumx2 - sumx*sumx);
    if (denom == 0)
    {
        // singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
        return 1;
    }

    *m = (n * sumxy  -  sumx * sumy) / denom;
    *b = (sumy * sumx2  -  sumx * sumxy) / denom;

    return 0;
}
