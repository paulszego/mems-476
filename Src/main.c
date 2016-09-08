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
#include "arm_math.h"
#include "stm32l4xx_nucleo.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "Trace.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static int N = 100;


static void*    tmpHnd;
static void*    hmdHnd;
static void*    magHnd;
static void*    accHnd;

static float            magSensitivity;
static SensorAxesRaw_t  magMax = { INT16_MIN, INT16_MIN, INT16_MIN };
static SensorAxesRaw_t  magMin = { INT16_MAX, INT16_MAX, INT16_MAX };
static SensorAxesRaw_t  magAvg;

static float            accSensitivity;
//static SensorAxesRaw_t  accMax;
//static SensorAxesRaw_t  accMin;
//static SensorAxesRaw_t  accAvg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
void Error_Handler( void );
static void MX_GPIO_Init( void );
static void MX_USART2_UART_Init( void );

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void initialise_monitor_handles( void );

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
static void checkCalibration( bool force )
{
    DrvStatusTypeDef    status;
    SensorAxesRaw_t     axes;


    if ( !force && BSP_PB_GetState( BUTTON_USER ) )
    {
        return;
    }

    printf( "\n\nPress the Blue Button to complete calibration.\n" );
    for ( int i = 3; i > 0; --i )
    {
        trace_printf( "Starting in %d...\n", i );
        HAL_Delay( 1000 );
    }
    printf( "Calibrating....\n" );

    while ( BSP_PB_GetState( BUTTON_USER ) )
    {
        status = BSP_MAGNETO_Get_AxesRaw( magHnd, &axes );

        magMin.AXIS_X = min( magMin.AXIS_X, axes.AXIS_X );
        magMin.AXIS_Y = min( magMin.AXIS_Y, axes.AXIS_Y );
        magMin.AXIS_Z = min( magMin.AXIS_Z, axes.AXIS_Z );

        magMax.AXIS_X = max( magMax.AXIS_X, axes.AXIS_X );
        magMax.AXIS_Y = max( magMax.AXIS_Y, axes.AXIS_Y );
        magMax.AXIS_Z = max( magMax.AXIS_Z, axes.AXIS_Z );

        magAvg.AXIS_X = ( magMin.AXIS_X + magMax.AXIS_X ) / 2;
        magAvg.AXIS_Y = ( magMin.AXIS_Y + magMax.AXIS_Y ) / 2;
        magAvg.AXIS_Z = ( magMin.AXIS_Z + magMax.AXIS_Z ) / 2;
    }

    trace_printf( "Magnetometer calibrated.\n\n" );
    trace_printf( "    min X %7.2f  Y %7.2f  Z %7.2f\n",
          (float)magMin.AXIS_X * magSensitivity,
          (float)magMin.AXIS_Y * magSensitivity,
          (float)magMin.AXIS_Z * magSensitivity
      );
    trace_printf( "    avg X %7.2f  Y %7.2f  Z %7.2f\n",
          (float)magAvg.AXIS_X * magSensitivity,
          (float)magAvg.AXIS_Y * magSensitivity,
          (float)magAvg.AXIS_Z * magSensitivity
      );
    trace_printf( "    max X %7.2f  Y %7.2f  Z %7.2f\n",
          (float)magMax.AXIS_X * magSensitivity,
          (float)magMax.AXIS_Y * magSensitivity,
          (float)magMax.AXIS_Z * magSensitivity
      );

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

/* USER CODE END 0 */

int main( void )
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
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
    while ( 1 )
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        checkCalibration( false );

        SensorAxesRaw_t axes;
        int32_t         bx = 0, by = 0, bz = 0;
        int32_t         gx = 0, gy = 0, gz = 0;

        uint32_t        start = HAL_GetTick();
        for ( int i = 0; i < N; i++ )
        {
            status = BSP_MAGNETO_Get_AxesRaw( magHnd, &axes );
            bx += axes.AXIS_X - magAvg.AXIS_X;
            by += axes.AXIS_Y - magAvg.AXIS_Y;
            bz += axes.AXIS_Z - magAvg.AXIS_Z;

            status = BSP_ACCELERO_Get_AxesRaw( accHnd, &axes );
            gx += axes.AXIS_X;
            gy += axes.AXIS_Y;
            gz += axes.AXIS_Z;
        }
        uint32_t        t = HAL_GetTick() - start;

        //  Fix orientation - NED (North, East, Down).
        float   Bx = bx;
        float   By = by;
        float   Bz = bz;
        Bx = Bx * magSensitivity / (float)N;
        By = By * magSensitivity / (float)N;
        Bz = Bz * magSensitivity / (float)N;
        trace_printf( "                    Bx %7.2f  By %7.2f  Bz %7.2f\n", Bx, By, Bz );
        continue;

        //  Fix orientation - NED (North, East, Down).
        float   Gx = -gx;
        float   Gy = -gy;
        float   Gz = gz;
        //  Earth's magnetic field at its surface: 0.25–0.60 gauss.
        //  Translate our values into mG (about 600 max value).
        //  This is just for display!!!
        Gx = (float)gx * accSensitivity / (float)N;
        Gy = (float)gy * accSensitivity / (float)N;
        Gz = (float)gz * accSensitivity / (float)N;
        trace_printf( "                    Gx %7.2f  Gy %7.2f  Gz %7.2f\n", Gx, Gy, Gz );


        //  Roll angle, ϕ.
        //  Use ATAN, to limit result to +/- 90 degrees.
        //  TODO    gz == 0 ???
        float   phi = atanf( Gy / Gz ) * 180.0f * M_1_PI;

        float   sinPhi;
        float   cosPhi;
        arm_sin_cos_f32( phi, &sinPhi, &cosPhi );

        //  Pitch angle, θ.
        //  Use ATAN2, to limit result to +/- 180 degrees.
        float tmp   = ( Gy * sinPhi ) + ( Gz * cosPhi );
        float theta = atan2f( -Gx, tmp ) * 180.0f * M_1_PI;

        float   sinTheta;
        float   cosTheta;
        arm_sin_cos_f32( theta, &sinTheta, &cosTheta );

        //  Yaw angle, ψ.
//        float   num = ( Bz * sinPhi ) - ( By * cosPhi );
        float   num = ( By * cosPhi ) - ( Bz * sinPhi );
        float   den = ( Bx * cosTheta ) + ( By * sinTheta * sinPhi ) + ( Bz * sinTheta * cosPhi );
        float   psi = atan2f( num, den ) * 180.0f * M_1_PI;
        if ( psi < 0.0 )
        {
            psi += 360.0;
        }



//        BSP_LED_Toggle( LED2 );
//        printf( "Tmp: %f\n", temp );
//        printf( "Hum: %f\n", humd );
        float  hdg = atan2f( Bx, By ) * 180.0f * M_1_PI;
        if ( hdg < 0.0 )
        {
            hdg += 360.0;
        }

        trace_printf( "T %4d  Hdg %6.2f  Psi %6.2f  Phi %6.2f  The %6.2f\n\n",
                      t, hdg, psi, phi, theta );
    }
    /* USER CODE END 3 */
}

/** System Clock Configuration
 */
void SystemClock_Config( void )
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
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                                  | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1
                                  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_4 ) != HAL_OK )
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        Error_Handler();
    }

    __HAL_RCC_PWR_CLK_ENABLE();

    if ( HAL_PWREx_ControlVoltageScaling( PWR_REGULATOR_VOLTAGE_SCALE1 ) != HAL_OK )
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq() / 1000 );
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/* USART2 init function */
static void MX_USART2_UART_Init( void )
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
    if ( HAL_UART_Init( &huart2 ) != HAL_OK )
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
static void MX_GPIO_Init( void )
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
    HAL_GPIO_Init( B1_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( LD2_GPIO_Port, &GPIO_InitStruct );

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET );
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler( void )
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
void assert_failed( uint8_t* file, uint32_t line )
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
