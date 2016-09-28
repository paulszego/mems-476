/*
 * sphere-fit.c
 *
 *  Created on: 27Sep.,2016
 *      Author: paul
 */

#include "sphere-fit.h"
#include "Trace.h"

#define MAX_ITER    100
//#define TRACE_FIT

#if defined(TRACE) && defined(TRACE_FIT)
#define trace_fit trace_printf
static void trace_mtx( arm_matrix_instance_f32* m );
static void trace_dat( uint32_t rows, uint32_t cols, const float32_t* data );
#else
inline int __attribute__((always_inline))
trace_fit( const char* format __attribute__((unused)), ...)
{
    return 0;
}
inline void __attribute__((always_inline))
trace_mtx( arm_matrix_instance_f32* m __attribute__((unused)) )
{
}
inline void __attribute__((always_inline))
trace_dat(  uint32_t rows __attribute__((unused)),
            uint32_t cols __attribute__((unused)),
            const float32_t* data __attribute__((unused)) )
{

}
#endif



static float64_t    x, y, z, x2, y2, z2, xy, yz, zx;            //  i, i², i*j
static float64_t    x3, y3, z3, x2y, y2z, z2x, xy2, yz2, zx2;   //  i³, i²*j, i*j²
static int          M;

//  An extra 2N(N+1) floats.
//      N   bytes
//      4   120
//      5   160
//      6   336
//      7   448
//
static float32_t    dat_jtj[ 4 ][ 4 ];      //  Jᵀ x J
static float32_t    dat_jtr[ 4 ];           //  Jᵀ x r
static float32_t    dat_j_1[ 4 ][ 4 ];      //  (Jᵀ x J)⁻¹
static float32_t    dat_tmp[ 4 ];           //  Either Ji or delta.

//  Matrix wrappers for the arm_math functions.
static arm_matrix_instance_f32 mtx_jtj = { 4, 4, dat_jtj[0] };
static arm_matrix_instance_f32 mtx_j_1 = { 4, 4, dat_j_1[0] };
static arm_matrix_instance_f32 mtx_jtr = { 4, 1, dat_jtr };
static arm_matrix_instance_f32 mtx_tmp = { 4, 1, dat_tmp };



extern void sphere_fit_init( void )
{
    x   = y   = z   = 0;
    x2  = y2  = z2  = 0;
    x3  = y3  = z3  = 0;
    xy  = yz  = zx  = 0;
    x2y = y2z = z2x = 0;
    xy2 = yz2 = zx2 = 0;

    M = 0;
}

/**
 * Collect another data sample for the sphere-fitting.
 *
 * @param   data    a pointer to a 3-element data array (x, y, z)
 *
 * @return  true if the data was collected ok, false otherwise.
 */
extern void sphere_fit_data( float32_t* d )
{
    sphere_fit_pnt( d[0], d[1], d[2] );
}
extern void sphere_fit_pnt( float32_t px, float32_t py, float32_t pz )
{
    M += 1;

    float32_t   xx = px * px;
    float32_t   yy = py * py;
    float32_t   zz = pz * pz;

    x   += px;
    y   += py;
    z   += pz;
    x2  += xx;
    y2  += yy;
    z2  += zz;
    xy  += px * py;
    yz  += py * pz;
    zx  += pz * px;
    x3  += px * xx;
    y3  += py * yy;
    z3  += pz * zz;
    x2y += xx * py;
    y2z += yy * pz;
    z2x += zz * px;
    xy2 += px * yy;
    yz2 += py * zz;
    zx2 += pz * xx;
}

extern void     sphere_fit_mean( float32_t* data )
{
    data[0] = x / M;
    data[1] = y / M;
    data[2] = z / M;
}


/**
 * Complete the sphere-fitting with the samples provided.
 *
 * @param   beta    the resulting fit parameters: a, b, c, r.
 *
 * @return  true if the sphere-fitting converged, false otherwise.
 */
extern bool     sphere_fit_calc( float32_t* beta )
{
    float32_t   rms;                //  root-mean-square error
    arm_status  status;

    float32_t   a = beta[0],
                b = beta[1],
                c = beta[2],
                r = beta[3];

    for ( int i = 0; i < MAX_ITER; i++ )
    {
        float32_t   D = a*a + b*b + c*c - r*r;
        float32_t   p = - 2.0*( a*x + b*y + c*z ) + M*D;

        // -2[ ∑x3 +∑xy2 +∑xz2 -2b∑(xy) -2c∑(xz) +D∑x -a( 3∑x2+∑y2+∑z2-2(a∑x+b∑y+c∑z)+MD )]
        // −2[ ∑z3 +∑zx2 +∑zy2 -2a∑(zx) -2b∑(zy) +D∑z -c( ∑x2+∑y2-3∑z2-2(a∑x+b∑y+c∑z)+MD )]
        // −2[ ∑y3 +∑yx2 +∑yz2 -2a∑(yx) -2c∑(yz) +D∑y -b( ∑x2+3∑y2+∑z2-2(a∑x+b∑y+c∑z)+MD )]

        dat_jtr[0] = x3 + xy2 + z2x - 2.0*b*xy - 2.0*c*zx + D*x - a*( 3.0*x2 + y2 + z2 + p );
        dat_jtr[1] = y3 + x2y + yz2 - 2.0*a*xy - 2.0*c*yz + D*y - b*( x2 + 3.0*y2 + z2 + p );
        dat_jtr[2] = z3 + zx2 + y2z - 2.0*a*zx - 2.0*b*yz + D*z - c*( x2 + y2 + 3.0*z2 + p );
        dat_jtr[3] =  r * ( x2 + y2 + z2 + p );
        arm_mat_scale_f32( &mtx_jtr, -2.0, &mtx_jtr );

        dat_jtj[0][0] = x2 - 2.0*a*x + a*a*M;
        dat_jtj[0][1] = xy - b*x - a*y + a*b*M;
        dat_jtj[0][2] = zx - c*x - a*z + a*c*M;
        dat_jtj[0][3] = r * ( x - a*M );

        dat_jtj[1][1] = y2 - 2.0*b*y + b*b*M;
        dat_jtj[1][2] = yz - c*y - b*z + b*c*M;
        dat_jtj[1][3] = r * ( y - b*M );

        dat_jtj[2][2] = z2 - 2.0*c*z + c*c*M;
        dat_jtj[2][3] = r * ( z - c*M );

        dat_jtj[3][3] = r*r*M;

        dat_jtj[1][0] = dat_jtj[0][1];
        dat_jtj[2][0] = dat_jtj[0][2];
        dat_jtj[3][0] = dat_jtj[0][3];
        dat_jtj[2][1] = dat_jtj[1][2];
        dat_jtj[3][1] = dat_jtj[1][3];
        dat_jtj[3][2] = dat_jtj[2][3];
        arm_mat_scale_f32( &mtx_jtj, 4.0, &mtx_jtj );

        trace_fit( "\n%3d\n", i );
        trace_fit( "jtj:\n" );   trace_mtx( &mtx_jtj );
        trace_fit( "jtr:\n" );   trace_mtx( &mtx_jtr );

        //  Note: this trashes mtx_jtj, even though it's const!
        status = arm_mat_inverse_f32( &mtx_jtj, &mtx_j_1 );             //  (Jᵀ x J)⁻¹
        if ( status != ARM_MATH_SUCCESS )
        {
            return false;
        }
        status = arm_mat_mult_f32   ( &mtx_j_1, &mtx_jtr, &mtx_tmp );   //  (Jᵀ x J)⁻¹ x Jᵀ x r
        if ( status != ARM_MATH_SUCCESS )
        {
            return false;
        }

        //  Calculate power of delta. Note it's not true RMS.
        arm_power_f32( dat_tmp, 4, &rms );
        trace_fit( "pwr:%19.16f\n", rms );

        if ( rms < 0.00000001 )
        {
            trace_fit( "Done!\n" );
            trace_fit( "  b:\n" );
            trace_dat( 4, 1, beta );
            return true;
        }

        a -= dat_tmp[0];
        b -= dat_tmp[1];
        c -= dat_tmp[2];
        r -= dat_tmp[3];

        beta[0] = a;
        beta[1] = b;
        beta[2] = c;
        beta[3] = r;

        trace_fit( "  d:\n" );   trace_mtx( &mtx_tmp );
        trace_fit( "  b:\n" );   trace_dat( 4, 1, beta );
    }

    trace_printf( "Failed!\n" );
    return false;
}

#if defined(TRACE) && defined(TRACE_FIT)
static void trace_mtx( arm_matrix_instance_f32* m )
{
    trace_dat( m->numRows, m->numCols, m->pData );
}
static void trace_dat( uint32_t rows, uint32_t cols, const float32_t* data )
{
    for ( int r = 0; r < rows; r++ )
    {
        trace_printf( "    " );
        for ( int c = 0; c < cols; c++ )
        {
            trace_printf( "%19.16f  ", data[ ( r * cols ) + c ] );
        }
        trace_printf( "\n" );
    }
}
#endif
