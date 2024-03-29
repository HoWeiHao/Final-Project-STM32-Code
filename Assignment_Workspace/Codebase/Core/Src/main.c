/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) EE2028 Teaching Team
 ******************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"

//Initialize peripherals
void SystemClock_Config(void);
static void UART1_Init(void); //To initialize UART
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

//Defining constants
#define DOUBLE_PRESS_INTERVAL 250
#define EXTREME_LINEAR_ACC 10

//Initializing global variables
volatile int pressed = 0; //Track number of times button is pressed
int mode = 0; // Tracks current mode; Default is 0
int first_press = -100; //
int t1 = 0;
int energy = 10; //Amount of energy charged, start with 10 as we want drone to be fully stocked before entering battle
int mode_changed = 0; //Allow for immediate change in modes halfway through printing sensor data
int collided_time = -2000; //Tracks time of sending collide message to prevent spam

float accel_scaled_prev[3] = { 0 }; //To store the previous acceleration values so that we can track sudden change in acceleration that may indicate collision

//Expected to be printed many times to notify of the mode, hence defined as global variables
char standby_msg[] = "Entering STANDBY MODE\r\n";
char battle_msg[] = "Entering BATTLE MODE\r\n";
char energy_msg[32]; // No message exceed 32 characters
char travel_msg[32];
uint8_t location[14];
int loc_index = 0;

/*Initialize arrays for sensor data. Sensors, data and units array are initialized
 in the same corresponding order to facilitate the printing code later that uses for
 loop instead of hard coding the entire sensor data printing process. Allows for
 flexibility to print 4 or 6 sets of data for the 2 different modes too.*/

float readings[6]; // To store data for easy printing
char *sensors[6] = { "M", "G", "P", "H", "A", "T" }; // To contain sensor in the same order as readings[6] for easier printing
int exceeded_flag[6] = { 0, 0, 0, 0, 0, 0 }; //Flags when measurement exceeded threshold
float max[6] = { 4.0, 20.0, 1015.0, 101.0, 20.0, 32.0 }; // To configure the max values later
char *units[6] = { "gauss", "dps", "hPa", "%", "m/s^2", "°C" }; //To check the units later


void laser_shoot() {
	while (energy >= 5) {
		energy -= 5;
		sprintf(energy_msg, "Laser shot! Energy now = %d\r\n", energy);
		HAL_UART_Transmit(&huart1, (uint8_t*) energy_msg, strlen(energy_msg),
				0xFFFF);
	}
}

int travel() {
	int timer = uwTick;
	uint8_t received_msg[1] = { 0 };
	UART1_Init();
	while (received_msg[0] == 0) {
		HAL_UART_Receive(&huart1, received_msg, 1, 500);
		if (uwTick - timer > 5000) {
			break;
		}
	}

	if (received_msg[0] == 13 || loc_index == 13) {
		sprintf(travel_msg, "Traveling to %s\r\n", location);
		HAL_UART_Transmit(&huart1, (uint8_t*) travel_msg, strlen(travel_msg),
				0xFFFF);
		loc_index = 0;
		memset(location, 0, sizeof(location));
		return 1;
	}

	location[loc_index] = received_msg[0];
	loc_index++;

	if (received_msg[0] == 0) {
		return 0;
	}
	return 0;
}

void check_PB() {

	if ((pressed == 1) && ((uwTick - first_press) > DOUBLE_PRESS_INTERVAL)) {
		if (mode == 0) {
			//Enhancement: In STANDBY_MODE we can choose location for our drone to travel to from command centre
			//Start timer to timeout if location is not entered
			int timer = uwTick;

			//Prompt
			HAL_UART_Transmit(&huart1, (uint8_t*) "Type destination\r\n", strlen("Type destination/r/n"), 0xFFFF);

			//Type location within the 10 seconds
			while (uwTick - timer < 10000) {
				//If travel location has already been entered, no need to wait anymore
				if (travel() == 1){
					break;
				}
			}

		} else if (mode == 1) {
			energy += 3;

			//Enhancement: update on energy amount
			sprintf(energy_msg, "Energy = %d\r\n", energy);
			HAL_UART_Transmit(&huart1, (uint8_t*) energy_msg, strlen(energy_msg), 0xFFFF);

			//If energy adding exceeds capacity, limit it
			if (energy > 10) {
				energy = 10;
			}

			//check if have sufficient energy, if yes, shoot
			laser_shoot();
		}
		//Reset pressed
		pressed = 0;
	}
	if (pressed == 2) {
		//Changing modes between standby (0) and battle (1), and set mode changing flag
		mode_changed = 1;
		if (mode == 0) {
			mode = 1;
			HAL_UART_Transmit(&huart1, (uint8_t*) battle_msg, strlen(battle_msg), 0xFFFF);
		} else if (mode == 1) {
			mode = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*) standby_msg, strlen(standby_msg), 0xFFFF);
		} else if (mode == 2) {

			//Recover from "The Last of EE2028" and print bettle mode message
			mode = 1;
			HAL_UART_Transmit(&huart1, (uint8_t*) battle_msg,
					strlen(battle_msg), 0xFFFF);

			//Reset previous acceleration values to avoid delayed trigger
			accel_scaled_prev[0] = 0;
			accel_scaled_prev[1] = 0;
			accel_scaled_prev[2] = 0;
		}
		//Reset press
		pressed = 0;
	}
}

void last() {
	//Initialize counter to alternate actions
	int rescue_t1 = uwTick;
	int rescue_count = 0;

	//Initialize the messages used in "The Last of EE2028"
	char attacked_msg[] = "Drone Was Attacked! \r\n";
	char terminated_msg[] = "Drone terminated. Bye World!\r\n";

	//Loop
	while (rescue_count < 40 && mode == 2) {

		rescue_count++;

		//Toggle LED every 250ms to get 2Hz blinking
		BSP_LED_Toggle(LED2_Pin);

		if (rescue_count % 4 == 0) {
			HAL_UART_Transmit(&huart1, (uint8_t*) attacked_msg, strlen(attacked_msg), 0xFFFF);
		}

		if (rescue_count == 40) {
			HAL_UART_Transmit(&huart1, (uint8_t*) terminated_msg, strlen(terminated_msg), 0xFFFF);
			exit(0);
		}

		//Wait for 250ms
		while ((uwTick - rescue_t1) < 250) {
			//Check while waiting for quicker response time
			check_PB();
		}
		rescue_t1 = uwTick;
	}
}


//This function checks the orientation of the drone, whether it has flipped over or collided into an object
void check_status() {
	//Initialize arrays to store raw and scaled reads for gyroscope and accelerometer
	float gyro_measurements[3] = { 0 };
	float gyro_scaled_measurements[3];
	int16_t accel_measurements[3] = { 0 };
	float accel_scaled_measurements[3];

	//Get sensor readings
	BSP_GYRO_GetXYZ(gyro_measurements);
	BSP_ACCELERO_AccGetXYZ(accel_measurements);

	//Scale each reading
	for (int j = 0; j < 3; j++) {
		gyro_scaled_measurements[j] = (float) gyro_measurements[j] / 1000.0f;
		accel_scaled_measurements[j] = (float) accel_measurements[j] * (9.8 / 1000.0f);
	}

    //Check the rotation of the drone's horizontal plane
	int xy_plane_rotate = sqrt((gyro_scaled_measurements[0] * gyro_scaled_measurements[0]) + (gyro_scaled_measurements[1] * gyro_scaled_measurements[1]));

	//Check if the drone is upright, by checking if has high angular rotation and
	//sudden negative linear acceleration to its positive z-axis
	if ((xy_plane_rotate > 180) && (accel_scaled_measurements[2] < 0)) {
		//If yes, activate "The Last of EE2028"
		if (mode == 1){
		mode = 2;
		last();
		}
	}

    //Check if an opposing force has caused the drone to accelerate in the opposite direction suddenly
	if ((uwTick - collided_time) > 2000)

		//Check all 3 axis
		for (int i = 0; i < 3; i++) {
			float linear_acc_change = accel_scaled_measurements[i] - accel_scaled_prev[i];
			if (linear_acc_change < 0) {
				linear_acc_change = -linear_acc_change;
			}

			//Check if change in acceleration is normal
			if ((linear_acc_change > EXTREME_LINEAR_ACC) || (linear_acc_change < -EXTREME_LINEAR_ACC)) {
				char collided_msg[] = "The drone have collided with something!\r\n";
				HAL_UART_Transmit(&huart1, (uint8_t*) collided_msg,
						strlen(collided_msg), 0xFFFF);
				//printf("The drone have collided with something!\n");
				accel_scaled_prev[i] = accel_scaled_measurements[i];

				//Sets collided time to prevent spamming messages
				collided_time = uwTick;

				//If 1 axis already signals collision, no need to repeat
				break;
			}
		}
}

void monitoring() {
	int count = 0; //Count to check how many 500 intervals have passed
	int active_sensors = 4; // STANDBY_MODE is default, hence  4 sensors operating by default
	char readings_msg[50]; //Sensor message to be printed, no message is longer than 50 characters
	int no_warn = 0; //Counter for number of normal readings without warnings

	//Get magnetometer data
	int16_t magne_readings[3] = { 0 }; // array to store the x, y and z readings.
	float magne_scaled[3] = { 0 };
	BSP_MAGNETO_GetXYZ(magne_readings);
	//Scale readings from all axis
	for (int j = 0; j < 3; j++) {
		magne_scaled[j] = (float) magne_readings[j] / 1000.0f;
	}
	//Derive overall value
	readings[0] = sqrt((magne_scaled[0] * magne_scaled[0]) + (magne_scaled[1] * magne_scaled[1]) + (magne_scaled[2] * magne_scaled[2]));

	//Check if maximum readings have been exceeded
	if (readings[0] > max[0]) {
		exceeded_flag[0] = 1;
	}

	//Get gyroscope data
	float gyro_readings[3] = { 0 };	// array to store the x, y and z readings.
	float gyro_scaled[3] = { 0 };
	BSP_GYRO_GetXYZ(gyro_readings);
	//Scale readings from all axis
	for (int j = 0; j < 3; j++) {
		gyro_scaled[j] = (float) gyro_readings[j] / 1000.0f;
	}
	//Derive overall value
	readings[1] = sqrt((gyro_scaled[0] * gyro_scaled[0]) + (gyro_scaled[1] * gyro_scaled[1]) + (gyro_scaled[2] * gyro_scaled[2]));

	//Check if maximum readings have been exceeded
	if (readings[1] > max[1]) {
		exceeded_flag[1] = 1;
	}

	//Get pressure data and check maximum values
	readings[2] = BSP_PSENSOR_ReadPressure();
		if (readings[2] > max[2]) {
			exceeded_flag[2] = 1;
		}


	//Get humidity data and check maximum values
	readings[3] = BSP_HSENSOR_ReadHumidity();
	if (readings[3] > max[3]) {
		exceeded_flag[3] = 1;
	}

	//Check mode to see if the other two sensor data is needed
	if (mode == 1) {
		//All 6 sensors are activated for BATTLE_MODE
		active_sensors = 6;

		//Get accelerator readings
		int16_t accel_readings[3] = { 0 };
		float accel_scaled[3];
		BSP_ACCELERO_AccGetXYZ(accel_readings);
		//Scale readings from all axis
		for (int j = 0; j < 3; j++) {
			accel_scaled[j] = (float) accel_readings[j] * (9.8 / 1000.0f);

		}
		//Derive overall value
		readings[4] = sqrt((accel_scaled[0] * accel_scaled[0]) + (accel_scaled[1] * accel_scaled[1]) + (accel_scaled[2] * accel_scaled[2]));
		if (readings[4] > max[4]) {
			exceeded_flag[4] = 1;
		}

		//Get temperature data and check maximum values
		readings[5] = BSP_TSENSOR_ReadTemp();
		if (readings[5] > max[5]) {
			exceeded_flag[5] = 1;
		}
	}

	//Iterate over each active sensors and print out warnings or sensor data respectively
	for (int i = 0; i < active_sensors; i++) {
		//Flag for mode change, to switch mode quickly if needed
		if (mode_changed == 1) {
			mode_changed = 0;
			break;
		}
		if (exceeded_flag[i] == 1) {
			//Transmit exceed warning
			sprintf(readings_msg, "%s: %f %s, exceed threshold of %f %s \r\n", sensors[i], readings[i], units[i], max[i], units[i]);
			//Reset exceed flag
			exceeded_flag[i] = 0;
			HAL_UART_Transmit(&huart1, (uint8_t*) readings_msg, strlen(readings_msg), 0xFFFF);


		} else {
			//Increase counter if no warning is made
			no_warn++;
		}

		//If no warning is made, can transmit sensor data as per normal
		if (no_warn == active_sensors) {

			for (int i = 0; i < active_sensors; i++) {
				//Need to format new messages for different sensors
				sprintf(readings_msg, "%s: %f (%s) ", sensors[i], readings[i], units[i]);
				HAL_UART_Transmit(&huart1, (uint8_t*) readings_msg, strlen(readings_msg), 0xFFFF);
			}

			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", strlen("\r\n"), 0xFFFF);
		}
	}


	//Put blocking code at the back to ensure everything runs fast first
	while (count < 2) {
		//Wait for 500ms intervals
		while ((uwTick - t1) < 500) {
			//Check while waiting to minimize lag time
			check_PB();
			check_status();
		}
		t1 = uwTick;

		//Increase interval count
		count++;

		//Toggle LED if BATTLE_MODE
		if (mode == 1) {
			//Toggle LED every 0.5 second
			BSP_LED_Toggle(LED2_Pin);
		}
	}
}

void standby_mode() {
	//LED ON throughout
	BSP_LED_On(LED2_Pin);
	monitoring();
}

void battle_mode() {
	//Check if enough energy the moment BATTLE_MODE starts
	laser_shoot();
	monitoring();
}

int main(void) {
	//Start timing immediately
	t1 = uwTick;

//Initialize all the sensors
	BSP_LED_Init(LED2_Pin);
	BSP_GYRO_Init();
	BSP_ACCELERO_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_TSENSOR_Init();

//Initialize HAL, UART and GPIO interrupt functions
	HAL_Init();
	UART1_Init();
	BSP_PB_Init(GPIO_PIN_13, BUTTON_MODE_EXTI);

	//Transmit STANDBY_MODE as it is default
	HAL_UART_Transmit(&huart1, (uint8_t*) standby_msg, strlen(standby_msg), 0xFFFF);
	while (1) {
		//Select mode
		if (mode == 0) {
			standby_mode();
		} else if (mode == 1) {
			battle_mode();
		}
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//Check if correct pin is triggered
	if (GPIO_Pin == BUTTON_EXTI13_Pin) {
		//Reset if press is too long from previous press or if double press is done
		if ((uwTick - first_press >= DOUBLE_PRESS_INTERVAL || pressed > 1)) {
			pressed = 0;
		}
	}
	first_press = uwTick;
//Update number of presses
	pressed++;
}

static void UART1_Init(void) {
	//UART pin configuration
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//Configuring UART1
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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		while (1)
			;
	}
}


