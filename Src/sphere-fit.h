/*
 * sphere.h
 *
 *  Created on: 27Sep.,2016
 *      Author: paul
 */

#ifndef SPHERE_FIT_H_
#define SPHERE_FIT_H_

#include "stm32l4xx_hal.h"

#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "arm_math.h"

/**
 * Initialize the sphere-fitting context.
 *
 * @return  true if the sphere-fitting is ready to accept data,
 *          false otherwise.
 */
extern void     sphere_fit_init( void );

/**
 * Collect another data sample for the sphere-fitting.
 *
 * @param   data    a pointer to a 3-element data array (x, y, z)
 *
 * @return  true if the data was collected ok, false otherwise.
 */
extern void     sphere_fit_data( float32_t* data );
extern void     sphere_fit_pnt( float32_t x, float32_t y, float32_t z );

/**
 * Get the mean values for each dimension (x, y, z)
 *
 * @param data
 */
extern void     sphere_fit_mean( float32_t* data );

/**
 * Complete the sphere-fitting with the samples provided.
 *
 * @param   beta    the resulting fit parameters: a, b, c, r.
 *
 * @return  true if the sphere-fitting converged, false otherwise.
 */
extern bool     sphere_fit_calc( float32_t* beta );

#endif /* SPHERE_FIT_H_ */
