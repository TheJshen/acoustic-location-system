/*******************************************************************************
 * main.cpp 
 * by Jason Shen, James Smith and Luis Sanchez
 * 
 * Acoustic Location System Filtering
 * 
 * This C module implements the DSP filtering of an input signal for the 
 * STM33F446RE Nucleo board.. The input signal will be passed in through 
 * on board ADC connected to a transponder that receives accoustic waves 
 * delivered by three transponders at 3 different frequencies. The coeffecients 
 * used for the filter are defined in main.h and implement  bandpass filters for
 * 35kHz, 40kHz and 45kHz with a pass bandwith of 500Hz. 
 * 
 * This program will output the time difference of arrival of the three
 * varying frequency signals. the default for for output is:
 *
 * [ A vs B time difference, B vs C time difference, A vs C time difference ]
 *
 * Currently the algorithm will first FIR filter the data for each of the 3 
 * frequencies. Then it will use an in place moving average function to 
 * smooth the data. Finally, it will run a peak detection function and find the 
 * peak of each of the 3 filtered signals which will then be placed on an output
 * pin to be available for beagle bone on OpenROV.
 *
 *
 *
 *
 * TODO: 
 * 1. Test on different location  inputs from python script,
 * 2. Integrate ADC and filter code
 * 3. If current filtering and smoothing is innaccurate develop new algorithm
 * 4. Clean up code
*******************************************************************************/

/* Includes */
#include "main.h"

/* ADC variables */
ADC_HandleTypeDef hadc1;
DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;

enum { ADC_BUFFER_LENGTH = 16666 };
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];

uint32_t g_ADCValue;
int g_MeasurementNumber;

DMA_HandleTypeDef g_DmaHandle;

/* FIR filter variables */
static float firStateF[BLOCK_SIZE + NUM_TAPS - 1];
unsigned int blockSize = BLOCK_SIZE;
unsigned int numBlocks = NUM_SAMPLES/BLOCK_SIZE;



/* Function Prototypes */
float calculateSoundSpeedInWater(float temp, float salinity, float depth);

int singleThresholdDetection(const float sampleData[], int startPosition);

void filterAndSmooth(float signal[], float filterCoeffs[], int smoothingCoeff, float * filteredSignal);

void movAvg(float signal[]);

int findMax(float signal[]);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
static void ConfigureDMA(void);

/*------------------------------------
 * Hyperterminal configuration
 * 9600 bauds, 8-bit data, no parity
 ************************************/
Serial pc(SERIAL_TX, SERIAL_RX);

DigitalOut myled(LED1);  //Debug LED

// Things to do when the second half of the buffer is filled
// TODO: insert filter code
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
    
}

void DMA2_Stream4_IRQHandler()
{
    HAL_DMA_IRQHandler(&g_DmaHandle);
}

void ADC_IRQHandler()
{
    HAL_ADC_IRQHandler(&hadc1);
}

int main(void)
{
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DAC_Init();
    MX_USART1_UART_Init();
  
    ConfigureDMA();
    
    HAL_ADC_Start(&hadc1); // To run the ADC continuously
    //HAL_ADC_Start_IT(&hadc1); // To run the ADC with a interrupt handler
    //HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUFFER_LENGTH);
    
    printf("---------------------------------------------------------------- ");
    printf("                                                                 ");
    printf("    Underwater Acoustic Location System DSP\n");
    printf("                                                                 ");
    printf("---------------------------------------------------------------- ");
    printf("    Running Filter... ");
    printf("---------------------------------------------------------------- ");
    printf("    Number of Samples: %d\n", NUM_SAMPLES);
    printf("    Moving Average window: %d samples\n", MOV_AVG_WIND); 
    
    /*
    float velocity = calculateSoundSpeedInWater(temp, salinity, depth);
    pc.printf("The velocity is %lf", velocity);
    */
   
    // Fir instance variables 
    arm_fir_instance_f32 S1, S2, S3;
    arm_status status;
    float32_t *input, *output;
    
    // Fir Coefficients
    float coeffsA_f32[NUM_TAPS];
    float output_f32[NUM_SAMPLES];
    
    /* Input is the signal with all 3 frequencies present. */    
    input = &signalABC[0];
    output = &output_f32[0];
    
    /* The array of time differences to place on output pin */
    int toReturn[3]; 
        
    /***************************************************************************
    ****************** Filtering for f = 35kHz (from buoy A) *******************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 35kHz (buoy A)\n");
    }

    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S1, NUM_TAPS, (float *) &coeffsA[0], 
        &firStateF[0], blockSize);
    
    int i = 0;
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S1, input + (i * blockSize), 
                        output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
    
    /* Print before moving average */ 
    printf("----------Before moving average-------------\n");
    if(UALS_DEBUG) {
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    float * avgSignal[NUM_SAMPLES];
    // Calculate moving average and do peak detection; 
    movAvg(output);
   
    /* Print signal after moving average */ 
    printf("----------After moving average-------------\n");
    if(UALS_DEBUG) {
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    /* Find the maximum value of the filtered signal */
    int maxA = findMax(output);
    
    /***************************************************************************
    ********************* Filtering for f = 40kHz (from buoy B) ****************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 40kHz (buoy B)\n");
    }
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S2, NUM_TAPS, (float *) &coeffsB[0], 
        &firStateF[0], blockSize);
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S2, input + (i * blockSize), 
                        output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
    
    /* Print before moving average */ 
    if(UALS_DEBUG) {
        printf("----------Before moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    
    /* Perform moving average */
    movAvg(output);
   
    /* Print signal after moving average */ 
    if(UALS_DEBUG) {
        printf("----------After moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    /* Find the maximum value of the filtered signal */
    int maxB = findMax(output);


    /***************************************************************************
    ******************** Filtering for f = 45kHz (from buoy C) *****************
    ***************************************************************************/
    if(UALS_DEBUG) {
        printf("Beginning filtering for f = 45kHz (buoy C)\n");
    }
    /* Initialize FIR instance structure */
    arm_fir_init_f32(&S3, NUM_TAPS, (float *) &coeffsC[0], 
        &firStateF[0], blockSize);
    
    /* FIR Filtering */
    for( i = 0; i < numBlocks; i++) {
        arm_fir_f32(&S3, input + (i * blockSize), 
                        output + (i * blockSize), blockSize);
    }
    
    /* Take the absolute value of the filtered signal */
    for(i = 0; i < NUM_SAMPLES; i++ ) {
        if(output[i] < 0) {
            output[i] = -1 * output[i];    
        }
    } 
    
    /* Print before moving average */ 
    if(UALS_DEBUG) {
        printf("----------Before moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    
    /* Perform moving average */
    movAvg(output);
   
    /* Print signal after moving average */ 
    if(UALS_DEBUG) {
        printf("----------After moving average-------------\n");
        for(i = 0; i < NUM_SAMPLES; i++) {
            printf("%lf\n", output[i]);
        }
    }
    
    /* Find the maximum value of the filtered signal */
    int maxC = findMax(output);

    /* The time differences to be returned */
    /* TODO: should we take the absolute value? What does TDOA take in */
    float tdAB = maxB - maxA;
    float tdBC = maxB - maxC;
    float tdAC = maxC - maxA;

    printf("tdAB = %f\n", tdAB);
    printf("tdBC = %f\n", tdBC);
    printf("tdAC = %f\n", tdAC);

    toReturn[0] = tdAB;
    toReturn[1] = tdBC;
    toReturn[2] = tdAC;
}

/*
 * Function Name: singleThresholdDetection()
 * Function Declaration: int singleThresholdDetection(const float sampleData[],
                                                            int startPosition)
 * Function Description: Calculates the first occurrence of a value over the 
 * threshold value and returns the position.
 *
 * Params:
 *  arg1: sampleData - sample data to find position of first peak
 *  arg2: startPosition - threshold value to search for
 *
 * Return Value: position where threshold is first reached
 *
 */
int singleThresholdDetection(const float sampleData[], int startPosition)
{
    int i;
    
    for(i = startPosition; i < NUM_SAMPLES; i++) {
        if (sampleData[i] >= DETECT_THRESH) {
            return i;
        }
    }
    
    return  -1;
}


/* 
 * Function Name: movAvg();
 * Function Declaration: void movAvg(float signal[]);
 * Function Description: This function takes an input signal and does an in
 * place moving average algorithm. The window size is defined by MOV_AVG_WIND
 * in main.h. 
 *
 * This function serves a dual purpose of finding the position of the largest 
 * peak and returning that position.
 * 
 * Return Value: None
 *
 */
void movAvg(float signal[]) {
     int i = 0;
    
    int start = 0;
    int finish = 0;
    float total = 0;
    
    // Buffer to hold the most recent samples
    float buffer[MOV_AVG_WIND*2];
    
    // Go through signal array and calculate moving average
    for(i = 0; i < NUM_SAMPLES; i++) {
        start = i - MOV_AVG_WIND;
        finish = i + MOV_AVG_WIND;
        
        if(start < 0) {
            start = 0;
        }

        if(finish > NUM_SAMPLES) {
            finish = NUM_SAMPLES;
        }
        
        total = 0;
        for(int j = start; j < finish; j++) {
            total += signal[j];
        }
        
        if(i > MOV_AVG_WIND*2 - 1) {
            signal[i-(MOV_AVG_WIND*2)] = buffer[i % (MOV_AVG_WIND*2)];
        }

        // Rotating buffer to save the avg
        buffer[i%(MOV_AVG_WIND*2)] = total / (finish - start); 
            
    }
    for(int i = NUM_SAMPLES-MOV_AVG_WIND*2; i < NUM_SAMPLES; i++) { 
        signal[i] = buffer[i %(MOV_AVG_WIND*2)];
    }
    
    if(UALS_DEBUG) {
        for(int i = 0; i < NUM_SAMPLES; i++) {
            printf("averagedSignal[i] = %f\n", signal[i]);
        }
    }
}


/* 
 * Function Name: findMax();
 * Function Declaration: int findMax(float signal[]);
 * Function Description: This function takes an input signal and returns the
 * position of the maximum valued sample.
 *
 */
int findMax(float signal[]) {
    float maxVal = 0.0;
    int maxPosition = -1;
    int i;

    /* Go through each element searching for maximum */
    for(i = 0; i < NUM_SAMPLES; i++) {
        if(signal[i] > maxVal) {
            /* We've found a new max, replace */
            maxVal = signal[i];
            maxPosition = i;
        }
    }

    if(maxPosition > 0) {
        printf("Max position found at %d\n", maxPosition);
        return maxPosition;
    }
    else {
        printf("Error: no max found\n");
        return -1;
    }
}






/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  __ADC1_CLK_ENABLE();
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  //hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  //hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; //ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = DISABLE;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  //sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; //144
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

    DAC_ChannelConfTypeDef sConfig;

        /**DAC Initialization 
        */
    hdac.Instance = DAC;
    HAL_DAC_Init(&hdac);

        /**DAC channel OUT1 config 
        */
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOD_CLK_ENABLE();


  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */



void ConfigureDMA()
{
    __DMA2_CLK_ENABLE();
    g_DmaHandle.Instance = DMA2_Stream4;
  
    g_DmaHandle.Init.Channel  = DMA_CHANNEL_0;
    g_DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    g_DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_DmaHandle.Init.Mode = DMA_CIRCULAR;
    g_DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
    g_DmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
    g_DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    g_DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;
    g_DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
    HAL_DMA_Init(&g_DmaHandle);
    
    __HAL_LINKDMA(&hadc1, DMA_Handle, g_DmaHandle);
 
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);   
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}
/* USER CODE END 4 */
