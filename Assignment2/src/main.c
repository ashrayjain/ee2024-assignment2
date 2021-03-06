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

#define DEL_ASSET_TAG_ID 013
#define COUNT_DOWN_START 1
#define NOTE_PIN_HIGH() GPIO_SetValue(0, 1<<26);
#define NOTE_PIN_LOW()  GPIO_ClearValue(0, 1<<26);
#define NUM_OF_ACC_VALUES_TO_AVG 10
#define TEMP_SCALAR_DIV10 1
#define TEMP_NUM_HALF_PERIODS 340
#define TEMP_READ ((GPIO_ReadValue(0) & (1 << 2)) != 0)

// ##################################### //
// ######  Variable Definitions   ###### //
// ##################################### //

typedef enum {FFS_CALIBRATING, FFS_STDBY_COUNTING_DOWN, FFS_STDBY_ENV_TESTING, FFS_ACTIVE} SYSTEM_STATE;
typedef enum {UNKNOWN_TEMPERATURE, NORMAL, HOT} TEMPERATURE_STATE;
typedef enum {UNKNOWN_RADIATION, SAFE, RISKY} RADIATION_STATE;
typedef enum {NON_RESONANT, RESONANT} FLUTTER_STATE;
typedef enum {STDBY_MODE, ACTIVE_MODE} ACC_MODE;
typedef enum {EINT0, EINT1, ENT2, ENT3} EXTERNAL_INTERRUPT;
typedef enum {BUZZER_LOW, BUZZER_HIGH} BUZZER_STATE;
typedef enum {HANDSHAKE_NOT_DONE, HANDSHAKE_DONE} HANDSHAKE_STATE;

const unsigned short CALIBRATED_PORT = 1;
const unsigned short CALIBRATED_PIN = 31;

const unsigned short RESET_PORT = 0;
const unsigned short RESET_PIN = 4;

const unsigned short LIGHT_PORT = 2;
const unsigned short LIGHT_PIN = 5;

const unsigned short TEMP_UPPER_LIMIT = 340;

const unsigned short ACC_UPDATE_PERIOD_MS = 20;
const float ACC_THRESHOLD = 3;
const int FREQ_UPDATE_PERIOD_MS = 500;

const char *MESSAGE_READY = "RDY DEL_ASSET_TAG_ID\r\n";
const char *MESSAGE_HANDSHAKE_CONFIRM = "HSHK DEL_ASSET_TAG_ID\r\n";
const char *MESSAGE_CONFIRM_ENTER_CALIB = "CACK\r\n";
const char *MESSAGE_CONFIRM_ENTER_STDBY = "SACK\r\n";
const char *MESSAGE_REPORT_TEMPLATE = "REPT DEL_ASSET_TAG_ID %02d%s\r\n";

const char *REPLY_ACK = "RACK\r";
const char *REPLY_NOT_ACK = "RNACK\r";
const char *CMD_RESET_TO_CALIB = "RSTC\r";
const char *CMD_RESET_TO_STDBY = "RSTS\r";

unsigned short TIME_WINDOW_MS = 3000;
unsigned short UNSAFE_LOWER_HZ = 2;
unsigned short UNSAFE_UPPER_HZ = 10;
unsigned short REPORTING_PERIOD_MS = 1000;

const uint32_t notes[] = {
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

// #################################### //
// #####  Variable Declarations   ##### //
// #################################### //

SYSTEM_STATE currentState;

HANDSHAKE_STATE handShakeState;

short countDownStarted = 0;
int currentCountValue = 0;

TEMPERATURE_STATE temperatureState;
int32_t currentTemperatureReading;

uint32_t temp_t1 = 0;
uint32_t temp_t2 = 0;
int temp_i = 0;
int32_t *temp_pointer = 0;

RADIATION_STATE radiationState = SAFE;

ACC_MODE accMode;
int8_t accX = 0;
int8_t accY = 0;
int8_t accZ = 0;
int8_t accXOffset = 0;
int8_t accYOffset = 0;
int8_t accZOffset = 0;
int8_t accZValToRemove = 0;

int accValues[NUM_OF_ACC_VALUES_TO_AVG] = {0};
int accValuesSorted[NUM_OF_ACC_VALUES_TO_AVG] = {0};
int currentAccIdx = 0;
float currentAccZFilteredValue = 0.0;
float prevAccZFilteredValue = 0.0;
uint8_t hasCrossedAccThreshold = 0;

int currentFreqCounter = 0;
float currentFrequency = 0;

FLUTTER_STATE flutterState;
short isWarningOn = 0;
short setWarningToStop = 0;
short setWarningToStart = 0;
short sendReport = 0;

BUZZER_STATE buzzerState;

volatile uint32_t msTicks; // counter for 1ms SysTicks
volatile uint32_t oneSecondTick = 0;
volatile uint32_t accTick = 0;
volatile uint32_t warningTick = 0;
static 	 char* msg = NULL;

// #################################### //
// #####  Function Declarations   ##### //
// #################################### //

//// Initializers ////
void initAllPeripherals	(void);
static	void init_ssp	(void);
static	void init_i2c	(void);
static	void init_GPIO	(void);
static  void init_UART  (void);
		void init_buzzer(void);
		void init_timer (void);
		void init_temp_interrupt(int32_t *var);
		void init_handShake(void);

//// handlers ////

// state handlers
void calibratingHandler			();
void stdbyCountingDownHandler	();
void stdbyEnvTestingHandler		();
void activeHandler				();

// Interrupt handler helpers
void oneSecondHandler			();

// Interrupt helpers
uint32_t getMsTicks				(void);

//// State changers	////
void enterCalibratingState		();
void enterStdByCountingDownState();
void enterActiveState			();
void leaveCalibratingState		();
void leaveStdByCountingDownState();
void leaveActiveState			();

//// Sensor Readers ////
void updateReadings				();
void updateTemperatureReading	();
void updateAccReading			();

// Sensor Data Manipulators
void updateFreqCounter			();

//// Actuator Functions ////

// warning related
void startWarning 		();
void turnOnLedArray		();

void runWarning			();

void stopWarning 		();
void turnOffLedArray	();

// counting related
void countDownFrom	(int startCount);
void decrementCount	();

// oled related
void writeHeaderToOled	(char *str);
void writeStatesToOled	();
void writeTempToOled	();
void writeAccValueToOled();

// helper functions
static	void playNote		(uint32_t note, uint32_t durationMs);
void rgb_setLeds_OledHack	(uint8_t ledMask);
void tempToString			(char *str);
void toStringInt			(char *str, int val);
void toStringDouble			(char *str, float val);

// ########################### //
// ##### Implementations ##### //
// ########################### //

//-----------------------------------------------------------
//---------------------- Initialisers -----------------------
//-----------------------------------------------------------

void initAllPeripherals() {
	init_GPIO();
    init_i2c();
    init_ssp();
    init_UART();

    pca9532_init();
    pca9532_setLeds(0, 0xffff);

    acc_init();

	rgb_init();
	rgb_setLeds_OledHack(0);

	led7seg_init();
	led7seg_setChar('-', 0);

    oled_init();
    oled_clearScreen(OLED_COLOR_BLACK);

    light_init();
    light_enable();
    light_setRange(LIGHT_RANGE_64000);
    light_clearIrqStatus();

    init_buzzer();

    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_ClearPendingIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT0_IRQn);
}

static void init_ssp(void) {
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

static void init_i2c(void) {
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

static void init_GPIO(void) {
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

void init_buzzer() {
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

static void init_UART(void) {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);
}

void init_timer() {
	int preScaleValue2 = 10000;
	int preScaleValue1 = 100000;

	// configure Timer2 for acc reading

	TIM_TIMERCFG_Type TimerConfigStruct;
	TIM_MATCHCFG_Type TimerMatcher;

	TimerConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TimerConfigStruct.PrescaleValue = preScaleValue2;

	TimerMatcher.MatchChannel = 0;
	TimerMatcher.IntOnMatch = ENABLE;
	TimerMatcher.ResetOnMatch = TRUE;
	TimerMatcher.StopOnMatch = FALSE;
	TimerMatcher.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	TimerMatcher.MatchValue = (ACC_UPDATE_PERIOD_MS * 1000) / preScaleValue2;

	TIM_Init(LPC_TIM2, TIM_TIMER_MODE, &TimerConfigStruct);
	TIM_ConfigMatch (LPC_TIM2, &TimerMatcher);

	// configure Timer1 for frequency calculations

	TimerConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TimerConfigStruct.PrescaleValue = preScaleValue1;

	TimerMatcher.MatchChannel = 0;
	TimerMatcher.MatchValue = (TIME_WINDOW_MS * 1000) / preScaleValue1;

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TimerConfigStruct);
	TIM_ConfigMatch (LPC_TIM1, &TimerMatcher);

	//Configure NVIC

	NVIC_SetPriority(TIMER2_IRQn, ((0x01<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	NVIC_SetPriority(TIMER1_IRQn, ((0x01<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void init_temp_interrupt(int32_t *var) {
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

void init_handShake (void) {
	// enable UART interrupts to send/receive
	LPC_UART3->IER |= UART_IER_RBRINT_EN;
	LPC_UART3->IER |= UART_IER_THREINT_EN;
	NVIC_EnableIRQ(UART3_IRQn);
}

//-----------------------------------------------------------
//------------------------ Handlers -------------------------
//-----------------------------------------------------------

// state handlers

void calibratingHandler() {
	enterCalibratingState();
	while(currentState == FFS_CALIBRATING) {
		writeAccValueToOled();
        if (((GPIO_ReadValue(CALIBRATED_PORT) >> CALIBRATED_PIN) & 0x01) == 0) {
        	currentState = FFS_STDBY_COUNTING_DOWN;
        }
	}
	leaveCalibratingState();
}

void stdbyCountingDownHandler() {

	enterStdByCountingDownState();
	while(currentState == FFS_STDBY_COUNTING_DOWN || handShakeState == HANDSHAKE_NOT_DONE) {

	}
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
	enterActiveState();
	char report[25] = "";
	while(currentState == FFS_ACTIVE){
		// sanity check (due to reset interrupt which might have changed currentState)
		if (currentState == FFS_ACTIVE && !(temperatureState == NORMAL && radiationState == SAFE)) {
			currentState = FFS_STDBY_COUNTING_DOWN;
		}

		updateReadings();
		writeStatesToOled();
		// TODO: declare the array outside to remove overhead of getting new memory every iteration
		char freq[15] = "";
		sprintf(freq, "%.1f", currentFrequency);
		oled_putString(7, 40, freq, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

		if(setWarningToStop) {
			stopWarning();
			setWarningToStop = 0;
			isWarningOn = 0;
		}

		if(setWarningToStart) {
			startWarning();
			setWarningToStart = 0;
			isWarningOn = 1;
		}

		if (sendReport) {
			report[0] = '\0';
			sprintf(report, MESSAGE_REPORT_TEMPLATE, currentFrequency, isWarningOn ? " WARNING" : "");

			// TODO: UART to send report

		}

	}
	leaveActiveState();
}

// interrupt handlers
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
		if (isWarningOn) {
			switch(buzzerState) {
				case BUZZER_HIGH:
					NOTE_PIN_LOW();
					buzzerState = BUZZER_LOW;
					break;
				case BUZZER_LOW:
					NOTE_PIN_HIGH();
					buzzerState = BUZZER_HIGH;
					break;
			}
		}
		if (msTicks - accTick >= FREQ_UPDATE_PERIOD_MS) {
		  	accTick = msTicks;
			currentFrequency = ((currentFreqCounter*500.0)/FREQ_UPDATE_PERIOD_MS);
			int isNonResonant = (currentFrequency > UNSAFE_UPPER_HZ || currentFrequency < UNSAFE_LOWER_HZ);

			if (flutterState == RESONANT && isNonResonant) {
				TIM_ResetCounter(LPC_TIM1);
				flutterState = NON_RESONANT;
			} else if(flutterState == NON_RESONANT && !isNonResonant) {
				TIM_ResetCounter(LPC_TIM1);
				flutterState = RESONANT;
			}
			currentFreqCounter = 0;
		}
		break;
	default:
		break;
  }
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

void TIMER2_IRQHandler(void) {
	if(LPC_TIM2->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM2,TIM_MR0_INT);
		updateFreqCounter();
	}
}

void TIMER1_IRQHandler (void) {
	if(LPC_TIM1->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);

		if (flutterState == NON_RESONANT) {
			if (isWarningOn) {
				setWarningToStop = 1;
			}
		} else {
			if (!isWarningOn) {
				setWarningToStart = 1;
			}
		}
	}
}

void UART3_IRQHandler (void) {
	if (LPC_UART3->IIR & UART_IIR_INTID_RDA) {

	}
	if (LPC_UART3->IIR & UART_IIR_INTID_CTI) {
		
	}
	if (LPC_UART3->IIR & UART_IIR_INTID_THRE) {

	}
}

void oneSecondHandler() {
	if (countDownStarted) {
		decrementCount();
	}

	if (currentState == ACTIVE) {
		sendReport = 1;
	}
}

uint32_t getMsTicks(void) {
	return msTicks;
}

void disable_temp_interrupt() {
	LPC_GPIOINT->IO0IntEnF &= ~(1 << 2);
	LPC_GPIOINT->IO0IntEnR &= ~(1 << 2);
}

//-----------------------------------------------------------
//--------------------- State Changers ----------------------
//-----------------------------------------------------------

void enterCalibratingState() {
	disable_temp_interrupt();
	acc_setMode(ACC_MODE_MEASURE);
	TIM_ResetCounter(LPC_TIM2);
	TIM_Cmd(LPC_TIM2, ENABLE);

	writeHeaderToOled(" CALIBRATING! ");
	rgb_setLeds_OledHack(0);
	led7seg_setChar('-', 0);
}

void enterStdByCountingDownState() {
	TIM_Cmd(LPC_TIM2, DISABLE);
	acc_setMode(ACC_MODE_STANDBY);

	handShakeState = HANDSHAKE_NOT_DONE;
	countDownFrom(COUNT_DOWN_START);
	light_setHiThreshold(800);
	init_temp_interrupt(&currentTemperatureReading);

	init_handShake();

	writeHeaderToOled("   STAND-BY   ");
}

void enterActiveState() {
	acc_setMode(ACC_MODE_MEASURE);
	TIM_ResetCounter(LPC_TIM2);
	TIM_ResetCounter(LPC_TIM1);
	TIM_Cmd(LPC_TIM2, ENABLE);
	TIM_Cmd(LPC_TIM1, ENABLE);

	writeHeaderToOled("    ACTIVE    ");
	led7seg_setChar('-', 0);
}

void leaveActiveState() {
	TIM_Cmd(LPC_TIM1, DISABLE);
	if (isWarningOn) {
		stopWarning();
	}
}

void leaveCalibratingState() {
	accXOffset += accX;
	accYOffset += accY;
	accZOffset += accZ;
}

void leaveStdByCountingDownState() {
	countDownStarted = 0;
}



//-----------------------------------------------------------
//--------------------- Sensor Readers ----------------------
//-----------------------------------------------------------

void updateReadings() {
	switch(currentState) {
		case FFS_STDBY_ENV_TESTING:
			updateTemperatureReading();
			break;
		case FFS_ACTIVE:
			updateTemperatureReading();
			break;
		default:
			break;
	}
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
	accZValToRemove = accValues[currentAccIdx];
	accValues[currentAccIdx] = accZ;
	currentAccIdx = (++currentAccIdx) % NUM_OF_ACC_VALUES_TO_AVG;

	// average filter:
	//currentAccZFilteredValue = (prevAccZFilteredValue * NUM_OF_ACC_VALUES_TO_AVG - accZValToRemove + accZ) / NUM_OF_ACC_VALUES_TO_AVG;

	// median filter

	int tempIdx = 0;

	while(accValuesSorted[tempIdx++] != accZValToRemove);

	accValuesSorted[tempIdx] = accZ;

	int tempVal;
	if(accZValToRemove > accZ) {
		while ((--tempIdx) && accValuesSorted[tempIdx] < accZ) {
			tempVal = accZ;
			accZ = accValuesSorted[tempIdx];
			accValuesSorted[tempIdx] = tempVal;
		}
	} else if (accZ > accZValToRemove) {
		while (((++tempIdx) < NUM_OF_ACC_VALUES_TO_AVG) && (accValuesSorted[tempIdx] > accZ)) {
			tempVal = accZ;
			accZ = accValuesSorted[tempIdx];
			accValuesSorted[tempIdx] = tempVal;
		}
	}

	currentAccZFilteredValue = NUM_OF_ACC_VALUES_TO_AVG % 2 == 0 ? accValuesSorted[NUM_OF_ACC_VALUES_TO_AVG/2] :
	 		(accValuesSorted[NUM_OF_ACC_VALUES_TO_AVG/2] + accValuesSorted[(NUM_OF_ACC_VALUES_TO_AVG+1)/2])/2;

	if (hasCrossedAccThreshold == 1) {
		if ((prevAccZFilteredValue < 0 && currentAccZFilteredValue > 0) || (prevAccZFilteredValue > 0 && currentAccZFilteredValue < 0)) {
			currentFreqCounter ++;
			hasCrossedAccThreshold = 0;
		}
	}

	if (currentAccZFilteredValue <= -1 * ACC_THRESHOLD || currentAccZFilteredValue >= ACC_THRESHOLD) {
		hasCrossedAccThreshold = 1;
	}

	prevAccZFilteredValue = currentAccZFilteredValue;
}

//-----------------------------------------------------------
//------------------ Actuator Functions ---------------------
//-----------------------------------------------------------

void startWarning() {
	isWarningOn = 1;
	turnOnLedArray();
	rgb_setLeds_OledHack(RGB_RED);
	NOTE_PIN_HIGH();
}

void turnOnLedArray() {
	pca9532_setLeds(0xffff, 0xffff);
}

void stopWarning() {
	isWarningOn = 0;
	NOTE_PIN_LOW();
	buzzerState = BUZZER_LOW;
	turnOffLedArray();
	rgb_setLeds_OledHack(0);
	NOTE_PIN_LOW();
}

void turnOffLedArray() {
	pca9532_setLeds(0, 0xffff);
}

void runWarning() {
	//playNote(2272, 1);
}

//-----------------------------------------------------------
//-------------- Count-Down Related Functio -----------------
//-----------------------------------------------------------

void countDownFrom(int startCount) {
	currentState = FFS_STDBY_COUNTING_DOWN;
	currentCountValue = startCount;
	countDownStarted = 1;
}

void decrementCount() {
	if (currentCountValue != 0) {
		char str[2];
		toStringInt(str, currentCountValue--);
		led7seg_setChar(str[0], 0);
	}
	else {
		led7seg_setChar('0', 0);
		currentState = FFS_STDBY_ENV_TESTING;
		countDownStarted = 0;
	}
}

//-----------------------------------------------------------
//----------------- OLed Related Functio --------------------
//-----------------------------------------------------------

void writeHeaderToOled(char *str) {
	oled_clearScreen(OLED_COLOR_BLACK);
	oled_fillRect(0,0,96,23, OLED_COLOR_WHITE);
	oled_putString(7, 8, str, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
}

void writeStatesToOled() {
	char stateStrings[15] = "";
	strcat(stateStrings, tempStateStringMap[temperatureState]);
	strcat(stateStrings, radiationStateStringMap[radiationState]);
	oled_putString(7, 32, stateStrings, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void writeTempToOled() {
	char str[15] = "";
	tempToString(str);
	oled_putString(7, 48, str, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
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

//-----------------------------------------------------------
//-------------------- Helper Functions ---------------------
//-----------------------------------------------------------

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

void tempToString(char *str) {
	strcat(str, " TEMP: ");
	char tempStr[5] = "";
	toStringDouble(tempStr, currentTemperatureReading/10.0);
	strcat(str, tempStr);
	char unitStr[4] = {	(char)(128), 'C', ' ' };
	strcat(str, unitStr);
}

void toStringInt(char *str, int val) {
	sprintf(str, "%d", val);
}

void toStringDouble(char *str, float val) {
	sprintf(str, "%.1f", val);
}

void rgb_setLeds_OledHack(uint8_t ledMask) {
	rgb_setLeds(ledMask | RGB_GREEN);
}

int main (void) {
	// Setup SysTick Timer to interrupt at 1 msec intervals
	if (SysTick_Config(SystemCoreClock / 1000)) {
	    while (1);  // Capture error
	}

	initAllPeripherals();
	//init_timer();

	uint8_t data[10];
	uint32_t len = 0;
	uint8_t line[64];

	//test sending message
	msg = "Welcome to EE2024 \r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
	//test receiving a letter and sending back to port
	UART_Receive(LPC_UART3, data, 3, BLOCKING);
	data[3] = '\0';
	printf("%s\n", data);
	data[3] = '\n';
	data[4] = '\0';
	UART_Send(LPC_UART3, data, strlen(data), BLOCKING);
	//test receiving message without knowing message length
	/*len = 0;
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
	*/
	return 0;
	/*
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
    */
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
