/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    MODE_RANDOM,
    MODE_FULL_BUFFER,
	MODE_PREDICTIVE
} TransmissionMode;

TransmissionMode transmissionMode = MODE_FULL_BUFFER; // Default to random mode

// Define the FIFO capacity
#define SENSOR_BUFFER_CAPACITY 30

#define TRANSMISSION_INTERVAL 1000
uint32_t lastTransmissionTime = 0;

// FIFO structure for scalar (float) sensor data (e.g. temperature, humidity, pressure)
typedef struct {
    float data[SENSOR_BUFFER_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint32_t timestamp[SENSOR_BUFFER_CAPACITY];
} FIFO_Float;

// Structure for 3-axis vector data (e.g. accelerometer, magnetometer)
typedef struct {
    float x;
    float y;
    float z;
} Vector3;

// FIFO structure for vector sensor data
typedef struct {
    Vector3 data[SENSOR_BUFFER_CAPACITY];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint32_t timestamp[SENSOR_BUFFER_CAPACITY];
} FIFO_Vector;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Threshold definitions
#define HIGH_TEMP_THRESHOLD    27.0f    // High temp threshold in degC default 27
#define LOW_HUMIDITY_THRESHOLD 30.0f    // Humidity low threshold in %
#define HIGH_HUMIDITY_THRESHOLD 101.0f	// Humidity high threshold in %, set at 101 to disable, 70 as spec
#define VIBRATION_THRESHOLD_X    1.0f     // 1 m/s^2 is default threadhold. Using 11 for testing purposes.
#define VIBRATION_THRESHOLD_Y    1.0f
#define VIBRATION_THRESHOLD_Z    11.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// Global variable to hold the latest pressure reading (updated at ~25Hz)
volatile float latestPressureReading = 0.0f;

volatile uint8_t criticalEventFlag = 0;

// Global FIFO buffers for each sensor:
FIFO_Float fifoTemp;
FIFO_Float fifoHumidity;
FIFO_Float fifoPressure;
FIFO_Vector fifoAccel;
FIFO_Vector fifoMagneto;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */


uint8_t HTS221_Temp_Data_Ready(uint16_t DeviceAddr); //Temperature sensor
uint8_t HTS221_Hum_Data_Ready(uint16_t DeviceAddr); //Humidity sensor
uint8_t LPS22HB_Data_Ready(uint8_t DeviceAddr); //Pressure sensor
uint8_t LIS3MDL_Data_Ready(uint8_t DeviceAddr); //Magnetometer sensor
uint8_t LSM6DSL_Gyro_Data_Ready(uint8_t DeviceAddr); //Gyroscope sensor
uint8_t LSM6DSL_Acc_Data_Ready(uint8_t DeviceAddr); //Accelerometer sensor

void HandleCriticalEvent(void);

//Blue button triggers interrupt
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == BUTTON_EXTI13_Pin){
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		printf("\t Blue button is pressed.\r\n");
		toggleTransmissionMode();

	}
}

// FIFO push functions: they return -1 if the FIFO is full (i.e. new data is discarded).
int pushFIFO_Float(FIFO_Float *fifo, float value) {
    if (fifo->count >= SENSOR_BUFFER_CAPACITY) {
        // Buffer is full – discard new data
        return -1;
    }
    fifo->data[fifo->tail] = value;
    fifo->timestamp[fifo->tail] = HAL_GetTick();  // Save current timestamp
    fifo->tail = (fifo->tail + 1) % SENSOR_BUFFER_CAPACITY;
    fifo->count++;
    return 0;
}

int pushFIFO_Vector(FIFO_Vector *fifo, Vector3 value) {
    if (fifo->count >= SENSOR_BUFFER_CAPACITY) {
        return -1;
    }
    fifo->data[fifo->tail] = value;
    fifo->timestamp[fifo->tail] = HAL_GetTick();  // Save current timestamp
    fifo->tail = (fifo->tail + 1) % SENSOR_BUFFER_CAPACITY;
    fifo->count++;
    return 0;
}

// FIFO pop functions
int popFIFO_Float(FIFO_Float *fifo, float *value, uint32_t *timestamp) {
    if (fifo->count == 0) {
        return -1; // Empty
    }
    *value = fifo->data[fifo->head];
    *timestamp = fifo->timestamp[fifo->head];
    fifo->head = (fifo->head + 1) % SENSOR_BUFFER_CAPACITY;
    fifo->count--;
    return 0;
}

int popFIFO_Vector(FIFO_Vector *fifo, Vector3 *value, uint32_t *timestamp) {
    if (fifo->count == 0) {
        return -1;
    }
    *value = fifo->data[fifo->head];
    *timestamp = fifo->timestamp[fifo->head];
    fifo->head = (fifo->head + 1) % SENSOR_BUFFER_CAPACITY;
    fifo->count--;
    return 0;
}

void transmitEntireFIFO_Float(FIFO_Float *fifo, const char *sensorName) {
    float value;
    uint32_t timestamp;
    while (popFIFO_Float(fifo, &value, &timestamp) == 0) {
        // Transmit sensor name, value, and timestamp.
        printf("%s,%lu,%f\r\n", sensorName, timestamp, value);
    }
}

void transmitEntireFIFO_Vector(FIFO_Vector *fifo, const char *sensorName) {
    Vector3 value;
    uint32_t timestamp;
    while (popFIFO_Vector(fifo, &value, &timestamp) == 0) {
        // Transmit sensor name, X, Y, Z values and timestamp.
        printf("%s,%lu,%f,%f,%f\r\n", sensorName, timestamp, value.x, value.y, value.z);
    }
}


/*
 * ODR values:
 *
 * humidity/temp: 1Hz
 * accel/gyro: 52Hz
 * pressure: 25Hz
 * magnetometer: 40Hz
 */




extern void initialise_monitor_handles(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//To print to terminal
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void toggleTransmissionMode(void) {
    if (transmissionMode == MODE_RANDOM) {
        transmissionMode = MODE_FULL_BUFFER;
    } else if (transmissionMode == MODE_FULL_BUFFER) {
        transmissionMode = MODE_PREDICTIVE;
    } else if (transmissionMode == MODE_PREDICTIVE) {
        transmissionMode = MODE_RANDOM;
    }

    if (transmissionMode == MODE_RANDOM) {
        printf("Transmission mode toggled to: RANDOM MODE\r\n");
    } else if (transmissionMode == MODE_FULL_BUFFER) {
        printf("Transmission mode toggled to: FULL BUFFER MODE\r\n");
    } else if (transmissionMode == MODE_PREDICTIVE) {
        printf("Transmission mode toggled to: PREDICTIVE MODE\r\n");
    }
}

void transmitRandomBuffer(void) {
    int choice = rand() % 5; // Randomly select one of the 5 sensor buffers
    switch (choice) {
        case 0:
            if (fifoTemp.count > 0) {
                transmitEntireFIFO_Float(&fifoTemp, "Temperature");
            }
            break;
        case 1:
            if (fifoHumidity.count > 0) {
                transmitEntireFIFO_Float(&fifoHumidity, "Humidity");
            }
            break;
        case 2:
            if (fifoPressure.count > 0) {
                transmitEntireFIFO_Float(&fifoPressure, "Pressure");
            }
            break;
        case 3:
            if (fifoAccel.count > 0) {
                transmitEntireFIFO_Vector(&fifoAccel, "Accelerometer");
            }
            break;
        case 4:
            if (fifoMagneto.count > 0) {
                transmitEntireFIFO_Vector(&fifoMagneto, "Magnetometer");
            }
            break;
        default:
            break;
    }
}

float getFIFO_timeToFillFloat(FIFO_Float *fifo) {
    if (fifo->count == 0) {
        return -1; // Empty
    }
	 float inputRate = (HAL_GetTick() - fifo->timestamp[fifo->tail]) / (fifo->count); // Get the rate of input/frequency
	 uint16_t remainingCapacity = SENSOR_BUFFER_CAPACITY - fifo->count; // get the remaining unoccupied buffer
	 return remainingCapacity / inputRate; // Calculate the time taken to fill the remaining cap
}
float getFIFO_timeToFillVector(FIFO_Vector *fifo) {
    if (fifo->count == 0) {
        return -1; // Empty
    }
	 float inputRate = (HAL_GetTick() - fifo->timestamp[fifo->tail]) / (fifo->count); // Get the rate of input/frequency
	 uint16_t remainingCapacity = SENSOR_BUFFER_CAPACITY - fifo->count; // get the remaining unoccupied buffer
	 return remainingCapacity / inputRate; // Calculate the time taken to fill the remaining cap
}



void transmitFullBuffers(void) {
    uint8_t threshold = (SENSOR_BUFFER_CAPACITY * 99) / 100; // 99% threshold.

    if (fifoTemp.count >= threshold) {
        transmitEntireFIFO_Float(&fifoTemp, "Temperature");
    }
    else if (fifoHumidity.count >= threshold) {
        transmitEntireFIFO_Float(&fifoHumidity, "Humidity");
    }
    else if (fifoPressure.count >= threshold) {
        transmitEntireFIFO_Float(&fifoPressure, "Pressure");
    }
    else if (fifoAccel.count >= threshold) {
        transmitEntireFIFO_Vector(&fifoAccel, "Accelerometer");
    }
    else if (fifoMagneto.count >= threshold) {
        transmitEntireFIFO_Vector(&fifoMagneto, "Magnetometer");
    }
}

char* getMostFilledBuffer(void) {
 float minTimeToFill = FLT_MAX;
 char* minBuffer = NULL;
    if (fifoTemp.count > 0) {
     float timeToFill = getFIFO_timeToFillFloat(&fifoTemp); // Use input frequency and occupancy to calculate time for remaining cap to fill
     if (minTimeToFill > timeToFill) {
      minTimeToFill = timeToFill;
      minBuffer = "Temperature";
     }
    }
    if (fifoHumidity.count > 0) {
     float timeToFill = getFIFO_timeToFillFloat(&fifoHumidity);
     if (minTimeToFill > timeToFill) {
      minTimeToFill = timeToFill;
      minBuffer = "Humidity";
     }
    }
    if (fifoPressure.count > 0) {
     float timeToFill = getFIFO_timeToFillFloat(&fifoPressure);
     if (minTimeToFill > timeToFill) {
      minTimeToFill = timeToFill;
      minBuffer = "Pressure";
     }
    }
    if (fifoMagneto.count > 0) {
     float timeToFill = getFIFO_timeToFillVector(&fifoMagneto);
     if (minTimeToFill > timeToFill) {
      minTimeToFill = timeToFill;
      minBuffer = "Magneto";
     }
    }
    if (fifoAccel.count > 0) {
     float timeToFill = getFIFO_timeToFillVector(&fifoAccel);
     if (minTimeToFill > timeToFill) {
      minTimeToFill = timeToFill;
      minBuffer = "Accelerometer";
     }
    }

    return minBuffer;
}




void transmitpredictiveBuffers(const char* selectedBuffer) {
 if (strcmp(selectedBuffer,"Temperature") == 0) {
  transmitEntireFIFO_Float(&fifoTemp, "Temperature");
 } else if (strcmp(selectedBuffer,"Humidity") == 0) {
  transmitEntireFIFO_Float(&fifoHumidity, "Humidity");
 } else if (strcmp(selectedBuffer,"Pressure") == 0) {
  transmitEntireFIFO_Float(&fifoPressure, "Pressure");
 }  else if (strcmp(selectedBuffer,"Accelerometer") == 0) {
  transmitEntireFIFO_Vector(&fifoAccel, "Accelerometer");
 } else if (strcmp(selectedBuffer,"Magneto") == 0) {
  transmitEntireFIFO_Vector(&fifoMagneto, "Magnetometer");
 }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	initialise_monitor_handles();
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  /* USER CODE BEGIN 2 */
  BSP_TSENSOR_Init();//Temperature init
  BSP_HSENSOR_Init();//Humidity init
  BSP_PSENSOR_Init();//Pressure init
  BSP_MAGNETO_Init();//Magnetometer init
  BSP_GYRO_Init();//Gyroscope init
  BSP_ACCELERO_Init();//Accelerometer init

  HAL_Delay(100);
  srand(HAL_GetTick());// Seed RNG


  int getRandomDelay(void)
  {
      return (rand() % 11) + 10;
  }

//  Add random error in sensor outputs
  float getRandomErrorFactor(void)
  {
      return ((float)rand() / (float)RAND_MAX) * 0.1f - 0.05f;
  }

  uint32_t now = HAL_GetTick();

  // Generate an initial random 10-20ms delay for each sensor
  uint32_t randDelayTempHum   = getRandomDelay();
  uint32_t randDelayAccelGyro = getRandomDelay();
  uint32_t randDelayPressure  = getRandomDelay();
  uint32_t randDelayMagneto   = getRandomDelay();

  // Initialize last poll timestamps to current time plus base period plus random delay.
//  uint32_t lastTempHumPoll   = now + 1000 + randDelayTempHum;  // 1Hz sensor
//  uint32_t lastAccelGyroPoll = now + 19   + randDelayAccelGyro;  // 52Hz sensor
//  uint32_t lastPressurePoll  = now + 40   + randDelayPressure;   // 25Hz sensor
//  uint32_t lastMagnetoPoll   = now + 25   + randDelayMagneto;    // 40Hz sensor
  uint32_t lastTempHumPoll   = now + (uint32_t)(1000.0 / 1.0)  + randDelayTempHum;   // 1Hz sensor
  uint32_t lastAccelGyroPoll = now + (uint32_t)(1000.0 / 52.0) + randDelayAccelGyro; // 52Hz sensor
  uint32_t lastPressurePoll  = now + (uint32_t)(1000.0 / 25.0) + randDelayPressure;  // 25Hz sensor
  uint32_t lastMagnetoPoll   = now + (uint32_t)(1000.0 / 40.0) + randDelayMagneto;   // 40Hz sensor


  /* USER CODE END 2 */

  while (1)
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  {

	uint32_t now = HAL_GetTick();
//	printf("HALTick %lu\r\n",now);

	if (criticalEventFlag)
		{
	         HandleCriticalEvent();
	         // After handling, clear the flag so that routine tasks resume.
	         criticalEventFlag = 0;
	     }
	else{
	 // Poll Humidity/Temperature sensor at ~1Hz (1000ms + random 10-20ms)


		if (now >= lastTempHumPoll)
		{
			uint32_t sensorReadTime = HAL_GetTick();
		    int randPollDelay = getRandomDelay();
//		    printf("%d",randPollDelay);
		    lastTempHumPoll = now + 1000 + randPollDelay;


		    float temp = BSP_TSENSOR_ReadTemp();
		    float humidity = BSP_HSENSOR_ReadHumidity();

		    // Add error to readings
		    float tempError = getRandomErrorFactor();
		    float humError  = getRandomErrorFactor();
		    float newTemp = temp * (1.0f + tempError);
		    float newHumidity = humidity * (1.0f + humError);

		    if (newHumidity > 100.0f) {
		         newHumidity = 100.0f;
		    }

		    if (pushFIFO_Float(&fifoTemp, newTemp) != 0)
		         printf("Temperature FIFO full, discarding reading.\r\n");
		    if (pushFIFO_Float(&fifoHumidity, newHumidity) != 0)
		         printf("Humidity FIFO full, discarding reading.\r\n");

		    // Alert conditions with delay logging:
		    if (newTemp > HIGH_TEMP_THRESHOLD)
		    {
		         uint32_t alertTime = HAL_GetTick();
		         uint32_t sensorDelay = alertTime - sensorReadTime + randPollDelay;
		         uint32_t responseDelay = sensorDelay;  // Modify if response time is measured differently
		         printf("** Alert: %lu High temperature alert: %f C. Sensor delay: %lu ms, Response delay: %lu ms. Activating cooler **\r\n",
		        		 alertTime,newTemp, sensorDelay, responseDelay);
		    }
		    if (newHumidity < LOW_HUMIDITY_THRESHOLD)
		    {
		         uint32_t alertTime = HAL_GetTick();
		         uint32_t sensorDelay = alertTime - sensorReadTime + randPollDelay;
		         uint32_t responseDelay = sensorDelay;
		         printf("** Alert: %lu Low humidity alert: %f%%! Sensor delay: %lu ms, Response delay: %lu ms. Activating Humidifier. **\r\n",
		        		 alertTime, newHumidity, sensorDelay, responseDelay);
		    }
		    if (newHumidity > HIGH_HUMIDITY_THRESHOLD)
		    {
		         uint32_t alertTime = HAL_GetTick();
		         uint32_t sensorDelay = alertTime - sensorReadTime + randPollDelay;
		         uint32_t responseDelay = sensorDelay;
		         printf("** Alert: %lu High humidity alert: %f%%. Sensor delay: %lu ms, Response delay: %lu ms **\r\n",
		        		 alertTime, newHumidity, sensorDelay, responseDelay);
		    }
		}

	    // Poll Accelerometer/Gyro at ~52Hz (19ms + random 10-20ms)
		if (now >= lastAccelGyroPoll)
		{
		    int randPollDelay = getRandomDelay();
		    lastAccelGyroPoll = now + 19 + randPollDelay;

		    // Capture accelerometer sensor reading timestamp
		    uint32_t accelSensorTime = HAL_GetTick();

		    int16_t accel_data_i16[3] = {0};
		    BSP_ACCELERO_AccGetXYZ(accel_data_i16);
		    float accel_data[3];
		    Vector3 accelReading;
		    for (int i = 0; i < 3; i++)
		    {
		         float error = getRandomErrorFactor();
		         accel_data[i] = (accel_data_i16[i] / 100.0f) * (1.0f + error);
		    }

		    accelReading.x = accel_data[0];
		    accelReading.y = accel_data[1];
		    accelReading.z = accel_data[2];
		    if (pushFIFO_Vector(&fifoAccel, accelReading) != 0)
		         printf("Accelerometer FIFO full, discarding reading.\r\n");

		    // Check for vibration alert condition and log delays:
		    if (fabs(accel_data[0]) > VIBRATION_THRESHOLD_X ||
		         fabs(accel_data[1]) > VIBRATION_THRESHOLD_Y ||
		         fabs(accel_data[2]) > VIBRATION_THRESHOLD_Z)
		    {
		         uint32_t alertTime = HAL_GetTick();
		         uint32_t sensorDelay = alertTime - accelSensorTime + randPollDelay;
		         uint32_t responseDelay = sensorDelay;
		         printf("** Alert: %lu Vibration warning! Sensor delay: %lu ms, Response delay: %lu ms **\r\n", alertTime, sensorDelay, responseDelay);
		         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		         HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		         criticalEventFlag = 1;
		    }
		}

	    // Poll Pressure sensor at ~25Hz (40ms + random 10-20ms)
	    if (now >= lastPressurePoll)
	    {
	         int randPollDelay = getRandomDelay();
	         lastPressurePoll = now + 40 + randPollDelay;
	         float pressure = BSP_PSENSOR_ReadPressure();
	         float error = getRandomErrorFactor();
	         float newPressure = pressure * (1.0f + error);
	         if (pushFIFO_Float(&fifoPressure, newPressure) != 0)
	                 printf("Pressure FIFO full, discarding reading.\r\n");
//	         printf("Pressure: %f hPa\r\n", newPressure);
	    }

	    // Poll Magnetometer sensor at ~40Hz (25ms + random 10-20ms)
	    if (now >= lastMagnetoPoll)
	    {
	         int randPollDelay = getRandomDelay();
	         lastMagnetoPoll = now + 25 + randPollDelay;
	         int16_t magneto_data[3] = {0};
	         BSP_MAGNETO_GetXYZ(magneto_data);
	         float newMagneto[3];
	         Vector3 magnetoReading;
	         for (int i = 0; i < 3; i++)
	             {
	                 float error = getRandomErrorFactor();
	                 newMagneto[i] = magneto_data[i] * (1.0f + error);
	             }
	         magnetoReading.x = newMagneto[0];
	         magnetoReading.y = newMagneto[1];
	         magnetoReading.z = newMagneto[2];
	         if (pushFIFO_Vector(&fifoMagneto, magnetoReading) != 0)
	             printf("Magnetometer FIFO full, discarding reading.\r\n");
//	         printf("Magnetometer: X: %f, Y: %f, Z: %f\r\n", newMagneto[0], newMagneto[1], newMagneto[2]);

	    }

        if (transmissionMode == MODE_FULL_BUFFER)
        {
            transmitFullBuffers();
        }
        if (transmissionMode == MODE_PREDICTIVE)
        {
        	transmitpredictiveBuffers(getMostFilledBuffer());
        }

        if (transmissionMode == MODE_RANDOM)
        {
            transmitRandomBuffer();
        }

        // For RANDOM mode: transmit one randomly selected buffer at fixed intervals.
//        if ((now - lastTransmissionTime) >= TRANSMISSION_INTERVAL)
//        {
//            if (transmissionMode == MODE_RANDOM)
//            {
//                transmitRandomBuffer();
//            }
//
//            lastTransmissionTime = now;
//        }
	}

	    // A short delay to avoid a busy loop.
//	    HAL_Delay(1);// default polling rate
//	    HAL_Delay(200);
//	    printf("=========\r\n");
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER  BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Critical Event Handling Function */
void HandleCriticalEvent(void)
{
    // Example: Flash LEDs and print a warning.
    printf("** Critical event detected! Pausing routine tasks. **\r\n");

    // Non-blocking LED flashing can be done via timing (here, for illustration, we use blocking delays)
      for (int i = 0; i < 5; i++)
      {
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
          HAL_Delay(100);  // short delay
          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
          HAL_Delay(100);
      }
      HAL_Delay(1000);
      printf("** Critical event resolved. Resuming normal operations **\r\n");
    // Additional critical event handling (e.g., immediate data transmission or system safety measures) can be done here.
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
