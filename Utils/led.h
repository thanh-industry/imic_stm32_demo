#ifndef _LED_H_
#define _LED_H_

#include "main.h"
#include "stdio.h"
#include "stdbool.h"


typedef enum {
	Off,
	On
}ledState_e;


typedef enum  {
	Low = GPIO_PIN_RESET,
	High = GPIO_PIN_SET
}ledActiveLevel_e;


typedef struct {
	uint16_t pin;
	ledActiveLevel_e activeLevel;
	ledState_e initialState;
	ledState_e state;
	GPIO_TypeDef *port;
}ledDev_st;


void ledInit(ledDev_st *_dev,
		GPIO_TypeDef* _port,
		uint16_t _pin,
		ledActiveLevel_e _activeLevel,
		ledState_e _initState);

ledState_e ledGetState(ledDev_st *_dev);

void ledSetState(ledDev_st *_dev, ledState_e state);

GPIO_TypeDef *ledPort(ledDev_st *_dev);

uint16_t ledPin(ledDev_st *_dev);

ledActiveLevel_e ledActiveLevel(ledDev_st *_dev);


ledState_e  ledInitialState(ledDev_st *_dev);

void ledOn(ledDev_st *_dev);


void ledOff(ledDev_st *_dev);

bool ledIsOn(ledDev_st *_dev);

bool ledIsOff(ledDev_st *_dev);

void ledReset(ledDev_st *_dev);

void ledToggle(ledDev_st *_dev);

void ledBlink(ledDev_st *_dev,
    uint8_t count,
    uint16_t onTimeMs,
    uint16_t offTimeMs,
    uint16_t leadOffTimeMs,
    uint16_t trailOffTimeMs);

void ledForceState(ledDev_st *_dev, ledState_e state);


#endif  // _LED_H_
