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
#define CELSIUS_SYMBOL_ASCII 128
#define HANDSHAKE_SYMBOL_ASCII 129
#define NOT_HANDSHAKE_SYMBOL_ASCII 130


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

HANDSHAKE_STATE currentHandshakeState;
short hasNewCommand = 0;

const char MESSAGE_READY[20] = "RDY 013\r\n";
const char MESSAGE_HANDSHAKE_CONFIRM[20] = "HSHK 013\r\n";
const char MESSAGE_CONFIRM_ENTER_CALIB[20] = "CACK\r\n";
const char MESSAGE_CONFIRM_ENTER_STDBY[20] = "SACK\r\n";
const char MESSAGE_REPORT_TEMPLATE[20] = "REPT 013 %02d%s\r\n";

const char REPLY_ACK[7] = "RACK";
const char REPLY_NOT_ACK[7] = "RNACK";
const char CMD_RESET_TO_CALIB[7] = "RSTC";
const char CMD_RESET_TO_STDBY[7] = "RSTS";

char stationCommand[7] = "";

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

int8_t accValues[NUM_OF_ACC_VALUES_TO_AVG] = {0};
int8_t accValuesSorted[NUM_OF_ACC_VALUES_TO_AVG] = {0};
int8_t currentAccIdx = 0;
float currentAccZFilteredValue = 0.0;
float prevAccZFilteredValue = 0.0;
uint8_t hasCrossedAccThreshold = 0;

int currentFreqCounter = 0;
float currentFrequency = 0;

FLUTTER_STATE flutterState;
short isWarningOn = 0;
short setWarningToStop = 0;
short setWarningToStart = 0;

BUZZER_STATE buzzerState;

char newReport[20] = "";
short reportBytesLeftToSend = 0;

volatile uint32_t msTicks; // counter for 1ms SysTicks
volatile uint32_t oneSecondTick = 0;
volatile uint32_t accTick = 0;
volatile uint32_t warningTick = 0;
static 	 char* msg = NULL;

// #################################### //
// #####  Function Declarations   ##### //
// #################################### //

//// Initializers ////
void initAllPeripherals();
static	void init_ssp	(void);
static	void init_i2c	(void);
static	void init_GPIO	(void);
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

// uart related

void sendUartReady();
void processUartCommand();

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

void init_uart() {
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	UART_Init(LPC_UART3, &uartCfg);
	UART_TxCmd(LPC_UART3, ENABLE);

	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);
	// enable UART interrupts to send/receive
	//LPC_UART3->IER |= UART_IER_THREINT_EN;
	LPC_UART3->FCR |= UART_FCR_TRG_LEV1;
	NVIC_EnableIRQ(UART3_IRQn);
}

void init_timer() {
	int preScaleValue1 = 100000;
	int preScaleValue2 = 10000;
	int preScaleValue3 = 100000;

	// configure Timer2 for acc reading and freq updating

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

	// configure Timer1 for reporting

	TimerConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TimerConfigStruct.PrescaleValue = preScaleValue1;

	TimerMatcher.MatchChannel = 0;
	TimerMatcher.MatchValue = (REPORTING_PERIOD_MS * 1000) / preScaleValue1;

	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TimerConfigStruct);
	TIM_ConfigMatch (LPC_TIM1, &TimerMatcher);

	//configure timer3 for uart

	TimerConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TimerConfigStruct.PrescaleValue = preScaleValue3;

	TimerMatcher.ResetOnMatch = TRUE;
	TimerMatcher.MatchChannel = 0;
	TimerMatcher.MatchValue = 50;

	TIM_Init(LPC_TIM3, TIM_TIMER_MODE, &TimerConfigStruct);
	TIM_ConfigMatch (LPC_TIM3, &TimerMatcher);

	//Configure NVIC

	NVIC_SetPriority(TIMER2_IRQn, ((0x11<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);

	NVIC_SetPriority(TIMER1_IRQn, ((0x01<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);

	NVIC_SetPriority(TIMER3_IRQn, ((0x01<<3)|0x01));
	NVIC_ClearPendingIRQ(TIMER3_IRQn);
	NVIC_EnableIRQ(TIMER3_IRQn);
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

void disable_temp_interrupt() {
	LPC_GPIOINT->IO0IntEnF &= ~(1 << 2);
	LPC_GPIOINT->IO0IntEnR &= ~(1 << 2);
}

void init_handShake (void) {
	sendUartReady();
	TIM_ResetCounter(LPC_TIM3);
	TIM_Cmd(LPC_TIM3, ENABLE);
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
	while(currentState == FFS_STDBY_COUNTING_DOWN){
		processUartCommand();
	}
	leaveStdByCountingDownState();
}

void stdbyEnvTestingHandler() {
	while(currentState == FFS_STDBY_ENV_TESTING){
		updateReadings();
		writeStatesToOled();
		writeTempToOled();
		processUartCommand();
		if (temperatureState == NORMAL && radiationState == SAFE && currentHandshakeState == HANDSHAKE_DONE) {
			currentState = FFS_ACTIVE;
		}
	}
}

void activeHandler() {
	enterActiveState();
	while(currentState == FFS_ACTIVE){
		updateReadings();
		//updateFreqCounter();
		writeStatesToOled();
		char freq[15] = "";
		sprintf(freq, "%.1f", currentFrequency);
		oled_putString(7, 40, freq, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		processUartCommand();

		// sanity check (due to reset interrupt which might have changed currentState)
		if (currentState == FFS_ACTIVE && !(temperatureState == NORMAL && radiationState == SAFE)) {
			currentState = FFS_STDBY_COUNTING_DOWN;
		}

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

		if (isWarningOn) {
			//playNote(2551, 50);
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
		if (msTicks - warningTick >= TIME_WINDOW_MS) {
			warningTick = msTicks;
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

void TIMER1_IRQHandler (void) {
	if(LPC_TIM1->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM1,TIM_MR0_INT);

		newReport[0] = '\0';
		sprintf	(newReport, MESSAGE_REPORT_TEMPLATE, (int) currentFrequency, (isWarningOn ? " WARNING" : ""));
		reportBytesLeftToSend = strlen(newReport);

		reportBytesLeftToSend -= UART_Send (LPC_UART3, newReport, reportBytesLeftToSend, NONE_BLOCKING);

		if(reportBytesLeftToSend > 0) {
				UART_IntConfig(LPC_UART3, UART_INTCFG_THRE, ENABLE);
		}
	}
}

void TIMER2_IRQHandler(void) {
	if(LPC_TIM2->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM2,TIM_MR0_INT);
		updateFreqCounter();
	}
}

void TIMER3_IRQHandler(void) {
	if(LPC_TIM3->IR & (1 << 0)) {
		TIM_ClearIntPending(LPC_TIM3,TIM_MR0_INT);
		if (currentHandshakeState != HANDSHAKE_DONE) {
			sendUartReady();
		}
	}
}

void UART3_IRQHandler (void) {
	if ((LPC_UART3->IIR & 14) == UART_IIR_INTID_RDA) {
		if(strlen(stationCommand) != 0) {
			stationCommand[0] = '\0';
		}

		UART_Receive(LPC_UART3, stationCommand, 4, BLOCKING);
		if(stationCommand[3] == '\r') {
			hasNewCommand = 1;
		}

		//char data[5];
		//UART_Receive(LPC_UART3, data, 4, BLOCKING);
		//data[4] = '\0';
		//printf("%s\n", data);

		stationCommand[4] = '\0';
		//printf("RDA: %s\n", stationCommand);
	}

	if ((LPC_UART3->IIR & 14) == UART_IIR_INTID_CTI) {
		int currentlen = strlen(stationCommand);
		if (currentlen == 6)
		{
			memset(stationCommand,0,strlen(stationCommand));
			currentlen = 0;
		}
		UART_Receive(LPC_UART3, stationCommand+currentlen, 1, BLOCKING);
		if (stationCommand[currentlen] == '\r')
		{
			stationCommand[currentlen] = '\0';
			hasNewCommand = 1;
			printf("%s\n", stationCommand);
		}
		else
		NVIC_ClearPendingIRQ(UART3_IRQn);
	}

	if ((LPC_UART3->IIR & UART_IIR_INTID_THRE) == UART_IIR_INTID_THRE) {
		reportBytesLeftToSend -= UART_Send(LPC_UART3, (newReport + strlen(newReport) - reportBytesLeftToSend), reportBytesLeftToSend, NONE_BLOCKING);

		if(reportBytesLeftToSend == 0) {
			UART_IntConfig(LPC_UART3, UART_INTCFG_THRE, DISABLE);
			newReport[0] = '\0';
		}
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

//-----------------------------------------------------------
//--------------------- State Changers ----------------------
//-----------------------------------------------------------

void enterCalibratingState() {
	acc_setMode(ACC_MODE_MEASURE);
	TIM_ResetCounter(LPC_TIM2);
	TIM_Cmd(LPC_TIM2, ENABLE);
	writeHeaderToOled(" CALIBRATING! ");
	rgb_setLeds_OledHack(0);
	led7seg_setChar('-', 0);
	disable_temp_interrupt();
}

void enterStdByCountingDownState() {
	TIM_Cmd(LPC_TIM2, DISABLE);
	writeHeaderToOled("   STAND-BY   ");
	countDownFrom(COUNT_DOWN_START);
	acc_setMode(ACC_MODE_STANDBY);
	light_setHiThreshold(800);
	init_temp_interrupt(&currentTemperatureReading);
	currentHandshakeState = HANDSHAKE_NOT_DONE;
	init_handShake();
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

	// mean filter:
	//currentAccZFilteredValue = (prevAccZFilteredValue * NUM_OF_ACC_VALUES_TO_AVG - accZValToRemove + accZ) / NUM_OF_ACC_VALUES_TO_AVG;

	// median filter

	int tempIdx = 0;
	int tempVal = 0;

	while(accValuesSorted[tempIdx++] != accZValToRemove);

	accValuesSorted[tempIdx] = accZ;

	if(accZValToRemove > accZ) {
		while ((--tempIdx) && accValuesSorted[tempIdx] < accZ) {
			tempVal = accZ;
			accZ = accValuesSorted[tempIdx];
			accValuesSorted[tempIdx] = tempVal;
		}
	} else {
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
//------------------- Actuator Functio ----------------------
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
	if (currentState != FFS_CALIBRATING) {
		char handShakeStr[15] = "              ";
		handShakeStr[13] = (currentHandshakeState == HANDSHAKE_DONE)?(char)(HANDSHAKE_SYMBOL_ASCII):(char)(NOT_HANDSHAKE_SYMBOL_ASCII);
		oled_putString(7, 0, handShakeStr, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	}
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
//-------------------- UART Related Functions ---------------------
//-----------------------------------------------------------

void sendUartReady() {
	UART_Send(LPC_UART3, MESSAGE_READY, strlen(MESSAGE_READY), NONE_BLOCKING);
}

void processUartCommand() {
	if (hasNewCommand)
	{
		hasNewCommand = 0;
		if (strcmpi(stationCommand, REPLY_ACK) ==0 && 
			(currentState == FFS_STDBY_COUNTING_DOWN || currentState == FFS_STDBY_ENV_TESTING))
		{
			currentHandshakeState = HANDSHAKE_DONE;
			// send hshk
			
		}
		else if (strcmpi(stationCommand, REPLY_NOT_ACK)==0 && 
			(currentState == FFS_STDBY_COUNTING_DOWN || currentState == FFS_STDBY_ENV_TESTING))
		{
			init_handShake();
		}
		else if (strcmpi(stationCommand, CMD_RESET_TO_CALIB)==0 && currentHandshakeState == HANDSHAKE_DONE)
		{
			currentState = FFS_CALIBRATING;
			currentHandshakeState = HANDSHAKE_NOT_DONE;
			UART_Send(LPC_UART3, MESSAGE_CONFIRM_ENTER_CALIB, strlen(MESSAGE_CONFIRM_ENTER_CALIB), NONE_BLOCKING);
		}
		else if (strcmpi(stationCommand, CMD_RESET_TO_STDBY)==0 && currentState == FFS_ACTIVE)
		{
			currentState = FFS_STDBY_COUNTING_DOWN;
			currentHandshakeState = HANDSHAKE_NOT_DONE;
			UART_Send(LPC_UART3, MESSAGE_CONFIRM_ENTER_STDBY, strlen(MESSAGE_CONFIRM_ENTER_STDBY), NONE_BLOCKING);
		}
		memset(stationCommand,0,strlen(stationCommand));
	}
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
	char unitStr[4] = {	(char)(CELSIUS_SYMBOL_ASCII), 'C', ' ' };
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
	init_timer();

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

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
