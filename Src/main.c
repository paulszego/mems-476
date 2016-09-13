/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "arm_math.h"
#include "stm32l4xx_nucleo.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "Trace.h"
#include "crc.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#pragma pack(push)
#pragma pack(2)
typedef struct
{
    uint16_t    stx;
    uint16_t    seq;
    int16_t     b[3];   //  Magnetometer
    int16_t     g[3];   //  Accelerometer
    int16_t     w[3];   //  Gyroscope
    uint16_t    crc;
} sample_t;
#pragma pack(pop)

#define SAMPLE_STX  0x5555


//static int N = 10;


static void*    tmpHnd;
static void*    hmdHnd;
static void*    magHnd;
static void*    accHnd;

static float            magSensitivity;
//static SensorAxesRaw_t  magMax = { INT16_MIN, INT16_MIN, INT16_MIN };
//static SensorAxesRaw_t  magMin = { INT16_MAX, INT16_MAX, INT16_MAX };
static SensorAxesRaw_t  magAvg;

static float            accSensitivity;
//static SensorAxesRaw_t  accMax;
//static SensorAxesRaw_t  accMin;
//static SensorAxesRaw_t  accAvg;



//  TODO    use C++ as a better C
//static const int                  M = 100;

#define M 100           //  10 times a second, 10 seconds
#define N 4

//  The values are arranged in row order
//  element (i, j) is stored at [i*numCols + j]
float                   cal_values[ M * 3 ];

float                   cal_resid[ M ];
float                   cal_param[ N ];
float                   cal_delta[ N ];

float                   cal_J[ M * N ];
float                   cal_JT[ N * M ];
float                   cal_JP[ N * M ];

float                   cal_TMP1[ N * N ];      //  16
float                   cal_TMP2[ N * N ];      //  16

arm_matrix_instance_f32 cal_x   =   {   M,  3,  cal_values  };
arm_matrix_instance_f32 cal_r   =   {   M,  1,  cal_resid   };

arm_matrix_instance_f32 cal_b   =   {   N,  1,  cal_param   };
arm_matrix_instance_f32 cal_d   =   {   N,  1,  cal_delta   };


arm_matrix_instance_f32 cal_j   =   {   M,  N,  cal_J       };
arm_matrix_instance_f32 cal_jt  =   {   N,  M,  cal_JT      };
arm_matrix_instance_f32 cal_jp  =   {   N,  M,  cal_JP      };

arm_matrix_instance_f32 cal_tmp1 =  {   N,  N,  cal_TMP1    };
arm_matrix_instance_f32 cal_tmp2 =  {   N,  N,  cal_TMP2    };



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles( void );
static void trace_matrix( arm_matrix_instance_f32* m );

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/**
 * Calculate the residual and partial derivatives for a value vector.
 *
 * @param   x       the sample values (x, y, z).
 * @param   b       the N parameters values (a, b, c, r).
 * @param   j       the N partial derivatives to be filled in.
 *
 * @return  the residual for this sample.
 */
static float fx( const float* x, const float* b, float* j  )
{
    float   pwr;

    arm_sub_f32( x, b, j, 3 );          //  J = X - B
    arm_power_f32( j, 3, &pwr );        //  pwr = (x-a)^2 + (y-b)^2 + (z-c)^2

    j[3] = b[3];                        //  df/dr = r
    arm_scale_f32( j, -2.0, j, 4 );     //  scale everything by -2

    //  (x-a)^2 + (y-b)^2 + (z-c)^2 - r^2
    return pwr - ( b[3] * b[3] );
}


#define MAX_ITER    100

static bool doLeastSquares( void )
{
    double      last_rms = 0;
    for ( int k = 0; k < MAX_ITER; k++ )
    {
        float   rms = 0.0;
        for ( int i = 0; i < M; i++ )
        {
            cal_resid[i] = fx( &cal_values[ i * 3 ], cal_param, &cal_J[ i * N ] );
            rms += cal_resid[i] * cal_resid[i];
        }
        rms /= M;
        arm_sqrt_f32( rms, &rms );

        if ( fabs( last_rms - rms ) < ( 2 * FLT_EPSILON ) )
        {
            trace_printf( "Done!\n" );
            trace_printf( "  b:\n" );
            trace_matrix( &cal_b );
            return true;
        }
        last_rms = rms;

//        trace_printf( "%3d  %19.16f\n", k, rms );
//        trace_printf( "  r:\n" );   trace_matrix( &cal_r );
//        trace_printf( "  j:\n" );   trace_matrix( &cal_j );

        //  TODO    check status of each call!
        arm_status  status;
        status = arm_mat_trans_f32( &cal_j, &cal_jt );              //  jt  Jᵀ
        status = arm_mat_mult_f32( &cal_jt, &cal_j, &cal_tmp1 );    //  m1  Jᵀ x J

//        trace_printf( " jt:\n" );   trace_matrix( &cal_jt );
//        trace_printf( " m1:\n" );   trace_matrix( &cal_tmp1 );

        //  Note: this trashes tmp1, even though it's const!
        status = arm_mat_inverse_f32( &cal_tmp1, &cal_tmp2 );       //  m2  (Jᵀ x J)⁻¹
        status = arm_mat_mult_f32( &cal_tmp2, &cal_jt, &cal_jp );   //  jp  (Jᵀ x J)⁻¹ x Jᵀ
        status = arm_mat_mult_f32( &cal_jp, &cal_r, &cal_d );       //  d   (Jᵀ x J)⁻¹ x Jᵀ x r
        arm_sub_f32( cal_param, cal_delta, cal_param, N );          //  b   b - d

//        trace_printf( " m2:\n" );   trace_matrix( &cal_tmp2 );
//        trace_printf( " jp:\n" );   trace_matrix( &cal_jp );
//        trace_printf( "  d:\n" );   trace_matrix( &cal_d );
//        trace_printf( "  b:\n" );   trace_matrix( &cal_b );
    }

    trace_printf( "Failed!\n" );
    return false;
}

static void trace_matrix( arm_matrix_instance_f32* m )
{
    for ( int r = 0; r < m->numRows; r++ )
    {
        trace_printf( "    " );
        for ( int c = 0; c < m->numCols; c++ )
        {
            trace_printf( "%19.16f  ", m->pData[ ( r * m->numCols ) + c ] );
        }
        trace_printf( "\n" );
    }
}

static void checkCalibration( bool force )
{
    DrvStatusTypeDef    status;
    SensorAxesRaw_t     axes;


    if ( !force && BSP_PB_GetState( BUTTON_USER ) )
    {
        return;
    }

    printf( "\n\nMagnetometer calibration.\n" );
    for ( int i = 3; i > 0; --i )
    {
        trace_printf( "Starting in %d...\n", i );
        HAL_Delay( 1000 );
    }
    printf( "Collecting data....\n" );

    //  Take a reading every 10 seconds until full.
    int     remaining   = M;
    int     index       = 0;
    while ( remaining > 0 )
    {
        HAL_Delay( 100 );
        status = BSP_MAGNETO_Get_AxesRaw( magHnd, &axes );
        if ( status != COMPONENT_OK )
        {
            //  TODO    error
            continue;
        }

        //  Convert to milli-Gauss.
        cal_values[ index++ ] = axes.AXIS_X * magSensitivity;
        cal_values[ index++ ] = axes.AXIS_Y * magSensitivity;
        cal_values[ index++ ] = axes.AXIS_Z * magSensitivity;
        --remaining;
        trace_printf( "%d\n", remaining );
    }

//    int index = 0;
//    cal_values[ index++ ] =  8.9129979413812240;
//    cal_values[ index++ ] = 13.2544157159778480;
//    cal_values[ index++ ] = 20.0881828807218770;
//    cal_values[ index++ ] =  7.2598045769044570;
//    cal_values[ index++ ] = 12.5417470253629300;
//    cal_values[ index++ ] = 20.2346573196046000;
//    cal_values[ index++ ] =  7.8738424666584930;
//    cal_values[ index++ ] = 12.8905587195475860;
//    cal_values[ index++ ] = 20.1216148013347000;
//    cal_values[ index++ ] =  7.4232683487079845;
//    cal_values[ index++ ] = 12.8675403169556370;
//    cal_values[ index++ ] = 20.3211186336997540;
//    cal_values[ index++ ] =  8.6775710169102320;
//    cal_values[ index++ ] = 11.1809115776406890;
//    cal_values[ index++ ] = 20.2495063004654400;
//    cal_values[ index++ ] =  9.0869435469348140;
//    cal_values[ index++ ] = 12.0536281136592760;
//    cal_values[ index++ ] = 19.8884044880361340;
//    cal_values[ index++ ] =  8.3811460486387240;
//    cal_values[ index++ ] = 12.9677740228016220;
//    cal_values[ index++ ] = 19.8809695876807000;
//    cal_values[ index++ ] =  7.9122793899494230;
//    cal_values[ index++ ] = 13.5679572316817560;
//    cal_values[ index++ ] = 20.0775710461557100;
//    cal_values[ index++ ] =  9.0608001018391470;
//    cal_values[ index++ ] = 12.4775654341670850;
//    cal_values[ index++ ] = 19.4844953719025260;
//    cal_values[ index++ ] =  8.5155725388313480;
//    cal_values[ index++ ] = 12.6619991843578640;
//    cal_values[ index++ ] = 20.1781024656281360;


    //  Setup initial guess: use the mean values.
    float   xm  = 0.0;
    float   ym  = 0.0;
    float   zm  = 0.0;
    float   rm  = 0.0;
    for ( int i = 0; i < M; i++ )
    {
        float   x = cal_values[ i * 3 + 0 ];
        float   y = cal_values[ i * 3 + 1 ];
        float   z = cal_values[ i * 3 + 2 ];
        xm += x;
        ym += y;
        zm += z;
    }
    xm /= M;
    ym /= M;
    zm /= M;
//    for ( int i = 0; i < M; i++ )
//    {
//        float   x = cal_values[ i * 3 + 0 ] - xm;
//        float   y = cal_values[ i * 3 + 1 ] - ym;
//        float   z = cal_values[ i * 3 + 2 ] - zm;
//        float   r;
//        arm_sqrt_f32( x * x + y * y + z * z, &r );
//        rm += r;
//    }
//    cal_param[3]    = rm / M;
    cal_param[3]    = 450;             //  average magnetic field in milli-Gauss.
    cal_param[0]    = xm;
    cal_param[1]    = ym;
    cal_param[2]    = zm;



    trace_printf( "Data collected, calibrating...\n\n" );
    doLeastSquares();



//    printf( "\nPlace the board face UP and press the blue button...\n" );
//    for ( int i = 3; i > 0; --i )
//    {
//        trace_printf( "Starting in %d...\n", i );
//        HAL_Delay( 1000 );
//    }
//    printf( "Hit it....\n" );
//    while ( BSP_PB_GetState( BUTTON_USER ) )
//    {
//        //  do nothing...
//    }
//
//    //  TODO    measure positive +1g
//    status = BSP_ACCELERO_Get_AxesRaw( accHnd, &axes );
//    float Gx = axes.AXIS_X * accSensitivity / 1000.0f;
//    float Gy = axes.AXIS_Y * accSensitivity / 1000.0f;
//    float Gz = axes.AXIS_Z * accSensitivity / 1000.0f;
//    printf( "+1g  x:%5.3f  y:%5.3f  z:%5.3f\n", Gx, Gy, Gz );
//
//    printf( "\nNow place the board face DOWN and press the blue button...\n" );
//    for ( int i = 3; i > 0; --i )
//    {
//        trace_printf( "Starting in %d...\n", i );
//        HAL_Delay( 1000 );
//    }
//    printf( "Hit it....\n" );
//    while ( BSP_PB_GetState( BUTTON_USER ) )
//    {
//        //  do nothing...
//    }
//
//    //  Measure positive -1g
//    status = BSP_ACCELERO_Get_AxesRaw( accHnd, &axes );
//    Gx = axes.AXIS_X * accSensitivity / 1000.0f;
//    Gy = axes.AXIS_Y * accSensitivity / 1000.0f;
//    Gz = axes.AXIS_Z * accSensitivity / 1000.0f;
//    printf( "-1g  x:%5.3f  y:%5.3f  z:%5.3f\n", Gx, Gy, Gz );
//
//    printf( "\nAccelerometer z-axis calibrated.\n\n" );
}


static volatile bool    txDone;

void HAL_UART_TxCpltCallback( UART_HandleTypeDef* huart )
{
    if ( huart == &huart4 )
    {
        txDone = true;
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
    sample_t    sample;
    uint16_t    sample_seq = 0u;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
    BSP_PB_Init( BUTTON_USER, BUTTON_MODE_GPIO );
    initialise_monitor_handles();

    DrvStatusTypeDef status;

    status = BSP_HUMIDITY_Init( HUMIDITY_SENSORS_AUTO, &hmdHnd );
    if ( status != COMPONENT_OK )
    {
        trace_printf( "Humidity init failed" );
    }
    status = BSP_HUMIDITY_Sensor_Enable( hmdHnd );
    if ( status != COMPONENT_OK )
    {
        trace_printf( "Humidity enable failed" );
    }

    status = BSP_TEMPERATURE_Init( TEMPERATURE_SENSORS_AUTO, &tmpHnd );
    if ( status != COMPONENT_OK )
    {
        trace_printf( "Temp init failed" );
    }
    status = BSP_TEMPERATURE_Sensor_Enable( tmpHnd );
    if ( status != COMPONENT_OK )
    {
        trace_printf( "Temp enable failed" );
    }

    status = BSP_MAGNETO_Init( MAGNETO_SENSORS_AUTO, &magHnd );
    status = BSP_MAGNETO_Sensor_Enable( magHnd );
    status = BSP_MAGNETO_Get_Sensitivity( magHnd, &magSensitivity );

    status = BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &accHnd );
    status = BSP_ACCELERO_Sensor_Enable( accHnd );
    status = BSP_ACCELERO_Get_Sensitivity( accHnd, &accSensitivity );

    float scale;
    status = BSP_MAGNETO_Get_FS( magHnd, &scale );
    trace_printf( "Mag Scale: %f\n", scale );
    trace_printf( "Mag  Sens: %f\n", magSensitivity );

    checkCalibration( true );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    txDone = true;
    while ( 1 )
    {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//        checkCalibration( false );

        SensorAxesRaw_t axes;
        int32_t         bx = 0, by = 0, bz = 0;
        int32_t         gx = 0, gy = 0, gz = 0;

        uint32_t        start = HAL_GetTick();
        for ( int i = 0; i < N; i++ )
        {
            status = BSP_MAGNETO_Get_AxesRaw( magHnd, &axes );
            bx += axes.AXIS_X;
            by += axes.AXIS_Y;
            bz += axes.AXIS_Z;

            status = BSP_ACCELERO_Get_AxesRaw( accHnd, &axes );
            gx += axes.AXIS_X;
            gy += axes.AXIS_Y;
            gz += axes.AXIS_Z;
        }
        uint32_t        t = HAL_GetTick() - start;

        sample.stx = SAMPLE_STX;
        sample.seq = sample_seq++;
        sample.b[0] = bx / N;
        sample.b[1] = by / N;
        sample.b[2] = bz / N;
        sample.g[0] = gx / N;
        sample.g[1] = gy / N;
        sample.g[2] = gz / N;
        sample.w[0] = 0;
        sample.w[1] = 0;
        sample.w[2] = 0;
        sample.crc  = crc16_ccitt( &sample, 22 );
//        HAL_UART_Transmit( &huart4, (uint8_t*)&sample, sizeof(sample), 1000 );

        while ( !txDone )
        {
            HAL_Delay( 5 );
        }

        txDone = false;
        HAL_UART_Transmit_IT( &huart4, (uint8_t*)&sample, sizeof(sample) );

//        //  Fix orientation - NED (North, East, Down).
//        float   Bx = bx;
//        float   By = by;
//        float   Bz = bz;
////        Bx = Bx * magSensitivity / (float)N;
////        By = By * magSensitivity / (float)N;
////        Bz = Bz * magSensitivity / (float)N;
////        trace_printf( "                    Bx %7.2f  By %7.2f  Bz %7.2f\n", Bx, By, Bz );
//        continue;
//
//        //  Fix orientation - NED (North, East, Down).
//        float   Gx = gx;
//        float   Gy = gy;
//        float   Gz = gz;
//        //  Earth's magnetic field at its surface: 0.25–0.60 gauss.
//        //  Translate our values into mG (about 600 max value).
//        //  This is just for display!!!
////        Gx = (float)gx * accSensitivity / (float)N;
////        Gy = (float)gy * accSensitivity / (float)N;
////        Gz = (float)gz * accSensitivity / (float)N;
////        trace_printf( "                    Gx %7.2f  Gy %7.2f  Gz %7.2f\n", Gx, Gy, Gz );
//
//
//        //  Roll angle, ϕ.
//        //  Use ATAN, to limit result to +/- 90 degrees.
//        //  TODO    gz == 0 ???
//        float   phi = atanf( Gy / Gz ) * 180.0f * M_1_PI;
//
//        float   sinPhi;
//        float   cosPhi;
//        arm_sin_cos_f32( phi, &sinPhi, &cosPhi );
//
//        //  Pitch angle, θ.
//        //  Use ATAN2, to limit result to +/- 180 degrees.
//        float tmp   = ( Gy * sinPhi ) + ( Gz * cosPhi );
//        float theta = atan2f( -Gx, tmp ) * 180.0f * M_1_PI;
//
//        float   sinTheta;
//        float   cosTheta;
//        arm_sin_cos_f32( theta, &sinTheta, &cosTheta );
//
//        //  Yaw angle, ψ.
////        float   num = ( Bz * sinPhi ) - ( By * cosPhi );
//        float   num = ( By * cosPhi ) - ( Bz * sinPhi );
//        float   den = ( Bx * cosTheta ) + ( By * sinTheta * sinPhi ) + ( Bz * sinTheta * cosPhi );
//        float   psi = atan2f( num, den ) * 180.0f * M_1_PI;
//        if ( psi < 0.0 )
//        {
//            psi += 360.0;
//        }
//
//
//
////        BSP_LED_Toggle( LED2 );
////        printf( "Tmp: %f\n", temp );
////        printf( "Hum: %f\n", humd );
//        float  hdg = atan2f( Bx, By ) * 180.0f * M_1_PI;
//        if ( hdg < 0.0 )
//        {
//            hdg += 360.0;
//        }
//
//        trace_printf( "T %4d  Hdg %6.2f  Psi %6.2f  Phi %6.2f  The %6.2f\n\n",
//                      t, hdg, psi, phi, theta );
    }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
//  huart4.Init.BaudRate = 115200;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while ( 1 )
    {
    }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 
