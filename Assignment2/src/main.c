/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *

 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "temp.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"

#include <stdio.h>
#include <string.h>


#define COUNT_DOWN_START 1

typedef enum {FFS_CALIBRATING, FFS_STDBY_COUNTING_DOWN, FFS_STDBY_ENV_TESTING, FFS_ACTIVE} STATES;
typedef enum {UNKNOWN_TEMPERATURE, NORMAL, HOT} TEMPERATURE_STATE;
typedef enum {UNKNOWN_RADIATION, SAFE, RISKY} RADIATION_STATE;
typedef enum {NON_RESONANT, RESONANT} FLUTTER_STATE;
typedef enum {STDBY_MODE, ACTIVE_MODE} ACC_MODE;
typedef enum {EINT0, EINT1, ENT2, ENT3} EXTERNAL_INTERRUPT;
typedef enum {true, false} bool;

const unsigned short CALIBRATED_PORT = 1;
const unsigned short CALIBRATED_PIN = 31;

const unsigned short RESET_PORT = 0;
const unsigned short RESET_PIN = 4;

const unsigned short LIGHT_PORT = 2;
const unsigned short LIGHT_PIN = 5;

const unsigned short TEMP_UPPER_LIMIT = 300;

const float ACC_THRESHOLD = 1;

int FREQ_UPDATE_TIME = 500;
unsigned short WARNING_UPDATE_TIME = 3000;
unsigned short UNSAFE_LOWER = 2;
unsigned short UNSAFE_UPPER = 10;


const char tempStateStringMap[3][9] = {
		"        ",
		" NORMAL ",
		"  HOT   "
};

const char radiationStateStringMap[3][7] = {
		"      ",
		"SAFE  ",
		"RISKY "
};

STATES currentState;
TEMPERATURE_STATE temperatureState;
int32_t currentTemperatureReading;

RADIATION_STATE radiationState = SAFE;
//int32_t currentRadiationReading;

int8_t accX = 0, accY = 0, accZ = 0;
int8_t accXOffset = 0, accYOffset = 0, accZOffset = 0;
int8_t accZValRemoved = 0;
float currentAccAvgValue = 0.0, prevAccAvgValue = 0.0;
uint8_t zeroCrossFlag = 0;

FLUTTER_STATE flutterState;
ACC_MODE accMode;

short countDownStarted = 0;
short isWarningOn = 0;

int currentCount;
int oneSecondTick = 0;

int currentFreqCounter = 0;
int countOfValuesToAverage = 4;
int currentAccIdx = 0;
int accValues[countOfValuesToAverage] = {0};
float currentFrequency = 0;
int accTick = 0;
int warningTick = 0;

static void playNote(uint32_t note, uint32_t durationMs);
void buzzer_init();
void calibratingHandler();
void stdbyCountingDownHandler();
void stdbyEnvTestingHandler();
void activeHandler();

void leaveCalibratingState();
void leaveActiveState();
void leaveStdByCountingDownState();

void updateTemperatureReading();
void updateAccReading();
void updateFreqCounter();

void countDownFrom(int startCount);
void decrementCount();
void toStringInt(char *str, int val);
void toStringDouble(char *str, float val);

void startWarning();
void stopWarning();
void runWarning();
void turnOnLedArray();
void turnOffLedArray();
void startWarningBuzzer();
void stopWarningBuzzer();

void writeHeaderToOled(char *str);
void writeTempToOled();
void tempToString(char *str);
void writeStatesToOled();
void updateReadings();
void writeAccValueToOled();

void configureTimer();

void rgb_setLeds_OledHack(uint8_t ledMask);

void rgb_setLeds_OledHack(uint8_t ledMask) {
	rgb_setLeds(ledMask | RGB_GREEN);
}

void oneSecondHandler();


// TEMP RELATED STUFF

#define TEMP_SCALAR_DIV10 1
#define TEMP_NUM_HALF_PERIODS 340
#define TEMP_READ ((GPIO_ReadValue(0) & (1 << 2)) != 0)

void temp_init_int(int32_t *var);


uint32_t temp_t1 = 0;
uint32_t temp_t2 = 0;
int temp_i = 0;
int32_t *temp_pointer = 0;

void temp_init_int(int32_t *var) {
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 2;
	PinCfg.Portnum = 0;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);

	GPIO_SetDir(0, (1<<2), 0);

	LPC_GPIOINT->IO0IntEnF |= (1 << 2);
	LPC_GPIOINT->IO0IntEnR |= (1 << 2);

	temp_pointer = var;
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

//////////////////////



// SYSTICK RELATED STUFF

volatile uint32_t msTicks; // counter for 1ms SysTicks

// ****************
//  SysTick_Handler
void SysTick_Handler(void) {
  msTicks++;
  switch (currentState) {
  	case FFS_STDBY_COUNTING_DOWN:
  		if (msTicks - oneSecondTick >= 1000) {
			oneSecondTick = msTicks;
			oneSecondHandler();
		}
		break;
	case FFS_ACTIVE:
		if (msTicks - accTick >= FREQ_UPDATE_TIME) {
		  	accTick = msTicks;
			currentFrequency = ((currentFreqCounter*500.0)/FREQ_UPDATE_TIME);
			currentFreqCounter = 0;
		}
		/*
		if (msTicks - warningTick >= WARNING_UPDATE_TIME) {
			warningTick = msTicks;
			int isNonResonant = (currentFrequency > UNSAFE_UPPER || currentFrequency < UNSAFE_LOWER);

			if (flutterState == NON_RESONANT) {
				if (isNonResonant) {
					if (isWarningOn) {
						stopWarning();
					}
				} else {
					flutterState = RESONANT;
				}
			} else {
				if (isNonResonant) {
					flutterState = NON_RESONANT;
				} else {
					if (!isWarningOn) {
						startWarning();
					}
				}
			}
		}*/
		break;
	default:
		break;
  }

}

void oneSecondHandler() {
	if (countDownStarted) {
		decrementCount();
	}
}

uint32_t getMsTicks(void) {
	return msTicks;
}

void updateReadings() {
	switch(currentState) {
		case FFS_STDBY_ENV_TESTING:
			updateTemperatureReading();
			break;
		case FFS_ACTIVE:
			//currentTemperatureReading = 1000;
			//updateTemperatureReading();
			updateAccReading();
			break;
		case FFS_CALIBRATING:
			updateAccReading();
		default:
			break;
	}
}

void writeHeaderToOled(char *str) {
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_fillRect(0,0,96,23, OLED_COLOR_WHITE);
	oled_putString(7, 8, str, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

void writeTempToOled() {
	char str[15] = "";
	tempToString(str);
	oled_putString(7, 48, str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void writeStatesToOled() {
	char stateStrings[15] = "";
	strcat(stateStrings, tempStateStringMap[temperatureState]);
	strcat(stateStrings, radiationStateStringMap[radiationState]);
	oled_putString(7, 32, stateStrings, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void writeAccValueToOled() {
	char str[15] = "";
	strcat(str, "   X : ");
	char val[5] = "";
	toStringInt(val, accX);
	strcat(str, val);
	strcat(str, "   ");
	oled_putString(7, 32, str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	str[0] = '\0';
	strcat(str, "   Y : ");
	val[0] = '\0';
	toStringInt(val, accY);
	strcat(str, val);
	strcat(str, "   ");
	oled_putString(7, 40, str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	str[0] = '\0';
	strcat(str, "   Z : ");
	val[0] = '\0';
	toStringInt(val, accZ);
	strcat(str, val);
	strcat(str, "   ");
	oled_putString(7, 48, str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void toStringInt(char *str, int val) {
	sprintf(str, "%d", val);
}

void toStringDouble(char *str, float val) {
	sprintf(str, "%.1f", val);
}

void tempToString(char *str) {
	strcat(str, " TEMP: ");
	char tempStr[5] = "";
	toStringDouble(tempStr, currentTemperatureReading/10.0);
	strcat(str, tempStr);
	char unitStr[4] = {	(char)(128), 'C', ' ' };
	strcat(str, unitStr);
}

void calibratingHandler() {
	writeHeaderToOled(" CALIBRATING! ");
	rgb_setLeds_OledHack(0);
	led7seg_setChar('-', 0);
	acc_setMode(ACC_MODE_MEASURE);
	while(currentState == FFS_CALIBRATING) {
		updateReadings();
		writeAccValueToOled();
        if (((GPIO_ReadValue(CALIBRATED_PORT) >> CALIBRATED_PIN) & 0x01) == 0) {
        	currentState = FFS_STDBY_COUNTING_DOWN;
        }
	}
	leaveCalibratingState();
}

void stdbyCountingDownHandler() {
	writeHeaderToOled("   STAND-BY   ");
	countDownFrom(COUNT_DOWN_START);
	acc_setMode(ACC_MODE_STANDBY);
	light_setHiThreshold(800);
	while(currentState == FFS_STDBY_COUNTING_DOWN){}
	leaveStdByCountingDownState();
}

void stdbyEnvTestingHandler() {
	while(currentState == FFS_STDBY_ENV_TESTING){
		updateReadings();
		writeStatesToOled();
		writeTempToOled();
		if (temperatureState == NORMAL && radiationState == SAFE) {
			currentState = FFS_ACTIVE;
		}
	}
}

void activeHandler() {
	writeHeaderToOled("    ACTIVE    ");
	led7seg_setChar('-', 0);
	acc_setMode(ACC_MODE_MEASURE);
	while(currentState == FFS_ACTIVE){
		//updateReadings();
		//updateFreqCounter();
		writeStatesToOled();
		char freq[15] = "";
		sprintf(freq, "%.1f", currentFrequency);
		oled_putString(7, 40, freq, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		//printf("%d\n", 30);
		// sanity check (due to reset interrupt which might have changed currentState)
		if (currentState == FFS_ACTIVE && !(temperatureState == NORMAL && radiationState == SAFE)) {
			currentState = FFS_STDBY_COUNTING_DOWN;
		}

		if (isWarningOn) {
			playNote(2551, 100);
		}
	}
	leaveActiveState();
}

void updateTemperatureReading() {
	if (currentTemperatureReading < TEMP_UPPER_LIMIT) {
		temperatureState = NORMAL;
	}
	else {
		temperatureState = HOT;
	}
}

void updateAccReading() {
	acc_read(&accX, &accY, &accZ);
	accX -= accXOffset;
	accY -= accYOffset;
	accZ -= accZOffset;
}

void updateFreqCounter() {
	updateAccReading();
	accZValRemoved = accValues[currentAccIdx];
	accValues[currentAccIdx] = accZ;
	currentAccIdx = (++currentAccIdx) % countOfValuesToAverage;
	currentAccAvgValue = (prevAccAvgValue * countOfValuesToAverage - accZValRemoved + accZ) / countOfValuesToAverage;

	if (zeroCrossFlag) {
		if ((prevAccAvgValue < 0 && currentAccAvgValue > 0) || (prevAccAvgValue > 0 && currentAccAvgValue < 0))) {
			currentFreqCounter ++;
			zeroCrossFlag = 0;
		}
	}

	if (currentAccAvgValue <= -1 * ACC_THRESHOLD || currentAccAvgValue >= ACC_THRESHOLD) {
		zeroCrossFlag = 1;
	}

	prevAccAvgValue = currentAccAvgValue;
}

void startWarning() {
	isWarningOn = 1;
	turnOnLedArray();
	rgb_setLeds_OledHack(RGB_RED);
}

void stopWarning() {
	isWarningOn = 0;
	turnOffLedArray();
	rgb_setLeds_OledHack(0);
}

void turnOnLedArray() {
	pca9532_setLeds(0xffff, 0xffff);
}

void turnOffLedArray() {
	pca9532_setLeds(0, 0xffff);
}

void runWarning() {
	//playNote(2272, 1);
}

void leaveActiveState() {
	if (isWarningOn) {
		stopWarning();
	}
}

void leaveStdByCountingDownState() {
	countDownStarted = 0;
}

void leaveCalibratingState() {
	accXOffset += accX;
	accYOffset += accY;
	accZOffset += accZ;
}

void countDownFrom(int startCount) {
	currentState = FFS_STDBY_COUNTING_DOWN;
	currentCount = startCount;
	countDownStarted = 1;
}

void decrementCount() {
	if (currentCount != 0) {
		char str[2];
		toStringInt(str, currentCount--);
		led7seg_setChar(str[0], 0);
	}
	else {
		led7seg_setChar('0', 0);
		currentState = FFS_STDBY_ENV_TESTING;
		countDownStarted = 0;
	}
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
	// Initialize button
	PINSEL_CFG_Type PinCfg;

	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 2;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);


	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = CALIBRATED_PIN;
	PinCfg.Portnum = CALIBRATED_PORT;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(CALIBRATED_PORT, (1<<CALIBRATED_PIN), 0);

	PinCfg.Funcnum = 0;
	PinCfg.Pinnum = 5;
	PinCfg.Portnum = 2;
	PinCfg.Pinmode = 0;
	PinCfg.OpenDrain = 0;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1<<5), 0);
	LPC_GPIOINT->IO2IntEnF |= (1 << 5);

	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 0;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void EINT0_IRQHandler(void) {
	currentState = FFS_CALIBRATING;
	LPC_SC->EXTINT |= (1 << EINT0);
}

void EINT3_IRQHandler(void) {
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) {
		if(radiationState == RISKY) {
			radiationState = SAFE;
			light_setHiThreshold(800);
			light_setLoThreshold(0);
		} else {
			radiationState = RISKY;
			light_setHiThreshold(62271);
			light_setLoThreshold(800);
		}
		light_clearIrqStatus();
		LPC_GPIOINT->IO2IntClr |= (1 << 5);
	}
	else if (((LPC_GPIOINT->IO0IntStatF >> 2) & 0x1) ||
			((LPC_GPIOINT->IO0IntStatR >> 2) & 0x1)) {
		if (temp_t1 == 0 && temp_t2 == 0) {
			temp_t1 = getMsTicks();
		}
		else if (temp_t1 != 0 && temp_t2 == 0) {
			temp_i++;
			if (temp_i == TEMP_NUM_HALF_PERIODS) {
				temp_t2 = getMsTicks();
				if (temp_t2 > temp_t1) {
					temp_t2 = temp_t2 - temp_t1;
				}
				else {
					temp_t2 = (0xFFFFFFFF - temp_t1 + 1) + temp_t2;
				}
				*temp_pointer = ((2*1000*temp_t2) / (TEMP_NUM_HALF_PERIODS*TEMP_SCALAR_DIV10) - 2731 );
				temp_t2 = 0;
				temp_t1 = 0;
				temp_i = 0;
			}
		}
		LPC_GPIOINT->IO0IntClr |= (1 << 2);
	}
}

void TIMER0_IRQHandler(void) {
	if(LPC_TIM0->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM0,TIM_MR0_INT);
		updateFreqCounter();
	} else if(LPC_TIM0->IR & (1 << 1)) {
		TIM_ClearIntPending(LPC_TIM0,TIM_MR1_INT);
		int isNonResonant = (currentFrequency > UNSAFE_UPPER || currentFrequency < UNSAFE_LOWER);

		if (flutterState == NON_RESONANT) {
			if (isNonResonant) {
				if (isWarningOn) {
					stopWarning();
				}
			} else {
				flutterState = RESONANT;
			}
		} else {
			if (isNonResonant) {
				flutterState = NON_RESONANT;
			} else {
				if (!isWarningOn) {
					startWarning();
				}
			}
		}
	}
}

void buzzer_init() {
	GPIO_SetDir(2, 1<<0, 1);
	GPIO_SetDir(2, 1<<1, 1);

	GPIO_SetDir(0, 1<<27, 1);
	GPIO_SetDir(0, 1<<28, 1);
	GPIO_SetDir(2, 1<<13, 1);
	GPIO_SetDir(0, 1<<26, 1);

	GPIO_ClearValue(0, 1<<27); //LM4811-clk
	GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
	GPIO_ClearValue(2, 1<<13); //LM4811-shutdn
}


void init_uart() {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	//pinsel_uart3();
	//supply power & setup working par.s for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}
void initAllPeripherals()
{
    init_i2c();
    init_ssp();
    init_GPIO();
    init_uart();

    pca9532_init();
    pca9532_setLeds(0, 0xffff);

    acc_init();

	rgb_init();
	rgb_setLeds_OledHack(0);

	led7seg_init();
	led7seg_setChar('-', 0);

    oled_init();
    oled_clearScreen(OLED_COLOR_BLACK);

    temp_init(getMsTicks);

    light_init();
    light_enable();
    light_setRange(LIGHT_RANGE_64000);
    light_clearIrqStatus();

    buzzer_init();

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_ClearPendingIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT0_IRQn);

    temp_init_int(&currentTemperatureReading);
}

void configureTimer() {
	TIM_TIMERCFG_Type TimerConfigStruct;
	TIM_MATCHCFG_Type TimerMatcher;

	TimerConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TimerConfigStruct.PrescaleValue = 10000;

	TimerMatcher.MatchChannel = 0;
	TimerMatcher.IntOnMatch = ENABLE;
	TimerMatcher.ResetOnMatch = TRUE;
	TimerMatcher.StopOnMatch = FALSE;
	TimerMatcher.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	TimerMatcher.MatchValue = 2;

	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TimerConfigStruct);
	TIM_ConfigMatch (LPC_TIM0, &TimerMatcher);

	TimerMatcher.MatchChannel = 1;
	TimerMatcher.MatchValue = 300;
	TIM_ConfigMatch (LPC_TIM0, &TimerMatcher);	

	NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	TIM_Cmd(LPC_TIM0, ENABLE);
}

static char* msg = NULL;

int main (void) {
	// Setup SysTick Timer to interrupt at 1 msec intervals
	if (SysTick_Config(SystemCoreClock / 1000)) {
	    while (1);  // Capture error
	}
	configureTimer();
	initAllPeripherals();

	/*uint8_t data = 0;
	uint32_t len = 0;
	uint8_t line[64];

	//test sending message
	msg = "Welcome to EE2024 \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	//test receiving a letter and sending back to port
	UART_Receive(LPC_UART3, &data, 1, BLOCKING);
	UART_Send(LPC_UART3, &data, 1, BLOCKING);
	//test receiving message without knowing message length
	len = 0;
	do
	{ 
		UART_Receive(LPC_UART3, &data, 1, BLOCKING);
		if (data != '\r')
		{
			len++;
			line[len-1] = data;
		}
	} while ((len<64) && (data != '\r'));
	line[len]=0;
	UART_SendString(LPC_UART3, &line);
	printf("--%s--\n", line);
	while (1);
	return 0;
*/
    while (1)
    {
    	switch(currentState)
    	{
			case FFS_CALIBRATING:
				calibratingHandler();
				break;
			case FFS_STDBY_COUNTING_DOWN:
				stdbyCountingDownHandler();
				break;
			case FFS_STDBY_ENV_TESTING:
				stdbyEnvTestingHandler();
				break;
			case FFS_ACTIVE:
				activeHandler();
				break;
			default:
				break;
    	}
        //Timer0_Wait(1);
    }
}

/*



*/

#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);

static uint32_t notes[] = {
        2272, // A - 440 Hz
        2024, // B - 494 Hz
        3816, // C - 262 Hz
        3401, // D - 294 Hz
        3030, // E - 330 Hz
        2865, // F - 349 Hz
        2551, // G - 392 Hz
        1136, // a - 880 Hz
        1012, // b - 988 Hz
        1912, // c - 523 Hz
        1703, // d - 587 Hz
        1517, // e - 659 Hz
        1432, // f - 698 Hz
        1275, // g - 784 Hz
};

static void playNote(uint32_t note, uint32_t durationMs) {

    uint32_t t = 0;

    if (note > 0) {

        while (t < (durationMs*1000)) {
            NOTE_PIN_HIGH();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            NOTE_PIN_LOW();
            Timer0_us_Wait(note / 2);
            //delay32Us(0, note / 2);

            t += note;
        }

    }
    else {
    	Timer0_Wait(durationMs);
        //delay32Ms(0, durationMs);
    }
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

