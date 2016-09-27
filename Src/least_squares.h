/*
 * least_squares.h
 *
 *  Created on: 13Sep.,2016
 *      Author: paul
 */

#ifndef LEAST_SQUARES_H_
#define LEAST_SQUARES_H_

//#include <stdbool.h>
//#include "arm_math.h"


#include "stm32l4xx_hal.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "arm_math.h"

//  The number of parameters to tune.
#ifndef LSQ_N
#define LSQ_N 4
#endif

/**
 * Interface for the optimization function.
 *
 * @param   i   input, the index of the data sample to evaluate.
 * @param   b   input, the N "beta" values, i.e. the parameters.
 * @param   j   output, the N Jacobian partial derivatives.
 *
 * @return  the residual for this data sample.
 */
typedef float32_t (*lsq_func)(
        uint32_t            i,
        const float32_t*    b,
        float32_t*          j );

/**
 *
 * @param   beta    the parameters to tune.
 * @param   M       the number of data samples.
 * @param   f       the evaluation function.
 *
 * @return  true if the optimization converged; false otherwise.
 */
extern bool lsq_optimize(
        float32_t*  beta,
        uint16_t    M,
        lsq_func    f );


#endif /* LEAST_SQUARES_H_ */
