/*
 * least_squares.c
 *
 *  Created on: 13Sep.,2016
 *      Author: paul
 */

#include "least_squares.h"
#include "Trace.h"

#define TRACE_LSQ
#define MAX_ITER    100

//  An extra 2*N*(N+1) floats.
static float32_t    dat_jtj[ LSQ_N ][ LSQ_N ];      //  Jᵀ x J
static float32_t    dat_jtr[ LSQ_N ];               //  Jᵀ x r
static float32_t    dat_j_1[ LSQ_N ][ LSQ_N ];      //  (Jᵀ x J)⁻¹
static float32_t    dat_tmp[ LSQ_N ];               //  Either Ji or delta.

//  Matrix wrappers for the arm_math functions.
arm_matrix_instance_f32 mtx_jtj = { LSQ_N, LSQ_N, dat_jtj[0] };
arm_matrix_instance_f32 mtx_j_1 = { LSQ_N, LSQ_N, dat_j_1[0] };
arm_matrix_instance_f32 mtx_jtr = { LSQ_N, 1, dat_jtr };
arm_matrix_instance_f32 mtx_tmp = { LSQ_N, 1, dat_tmp };


#if defined(TRACE) && defined(TRACE_LSQ)
#define trace_lsq trace_printf
static void trace_mtx( arm_matrix_instance_f32* m );
static void trace_dat( uint32_t rows, uint32_t cols, const float32_t* data );
#else
inline int __attribute__((always_inline))
trace_lsq( const char* format __attribute__((unused)), ...)
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


extern bool lsq_optimize(   float32_t*  beta,
                        uint16_t    M,
                        lsq_func    f )
{
    float32_t   r;                  //  rᵢ, the residual
    float32_t   rms;                //  root-mean-square error
    float32_t   last_rms = 0;       //  for comparison


    for ( int a = 0; a < MAX_ITER; a++ )
    {
        arm_fill_f32( 0, dat_jtj[0], LSQ_N * LSQ_N );
        arm_fill_f32( 0, dat_jtr, LSQ_N );

        rms = 0;
        for ( uint32_t i = 0; i < M; i++ )
        {
            r   = (*f)( i, beta, dat_tmp );
            rms = rms + ( r * r );

            for ( int j = 0; j < LSQ_N; j++ )
            {
                //  Update Jᵀ x rᵢ
                dat_jtr[j] += dat_tmp[j] * r;

                //  Update Jᵀ x J
                for ( int k = 0; k < LSQ_N; k++ )
                {
                    dat_jtj[j][k] += dat_tmp[j] * dat_tmp[k];
                }
            }
        }
        rms /= M;
        arm_sqrt_f32( rms, &rms );

        if ( fabs( last_rms - rms ) < ( 2 * FLT_EPSILON ) )
        {
            trace_lsq( "Done!\n" );
            trace_lsq( "  b:\n" );
            trace_dat( LSQ_N, 1, beta );
            return true;
        }
        last_rms = rms;

        trace_lsq( "%3d %19.16f\n", a, rms );
        trace_lsq( "jtj:\n" );   trace_mtx( &mtx_jtj );
        trace_lsq( "jtr:\n" );   trace_mtx( &mtx_jtr );

        //  TODO    check status of each call!
        arm_status  status;

//        arm_fill_f32( 0, dat_tmp, LSQ_N );

        //  Note: this trashes mtx_jtj, even though it's const!
        status = arm_mat_inverse_f32( &mtx_jtj, &mtx_j_1 );             //  (Jᵀ x J)⁻¹
        status = arm_mat_mult_f32   ( &mtx_j_1, &mtx_jtr, &mtx_tmp );   //  (Jᵀ x J)⁻¹ x Jᵀ x r
        arm_sub_f32( beta, dat_tmp, beta, LSQ_N );

        trace_lsq( "  d:\n" );   trace_mtx( &mtx_tmp );
        trace_lsq( "  b:\n" );   trace_dat( LSQ_N, 1, beta );
    }

    trace_printf( "Failed!\n" );
    return false;
}

#if defined(TRACE) && defined(TRACE_LSQ)
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
