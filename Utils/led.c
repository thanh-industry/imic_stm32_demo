/**
 *
 */

#include "led.h"


static void ledGPIOInit(ledDev_st *_dev)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = _dev->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(_dev->port, &GPIO_InitStruct);
}

void ledInit(ledDev_st *_dev, GPIO_TypeDef* _port, uint16_t _pin, ledActiveLevel_e _activeLevel, ledState_e _initState)
{
	if(_dev == NULL || _port == NULL)
	{
		return;
	}
	_dev->pin = _pin;
	_dev->port = _port;
	_dev->initialState =  _initState;
	_dev->activeLevel = _activeLevel;

	ledForceState(_dev, _initState);
	ledGPIOInit(_dev);
}




ledState_e ledGetState(ledDev_st *_dev)
{
    return _dev->state;
} 


void ledSetState(ledDev_st *_dev, ledState_e state)
{
    // Only set state if specified state is different from current state_.
    if (_dev->state != state)
    {
    	ledForceState(_dev, state);
    }
}

GPIO_TypeDef *ledPort(ledDev_st *_dev)
{
    return _dev->port;
}

uint16_t ledPin(ledDev_st *_dev)
{
    return _dev->pin;
}    


ledActiveLevel_e ledActiveLevel(ledDev_st *_dev)
{
    return _dev->activeLevel;
}


ledState_e  ledInitialState(ledDev_st *_dev)
{
    return _dev->initialState;
}


void ledOn(ledDev_st *_dev)
{
    ledSetState(_dev, On);
}


void ledOff(ledDev_st *_dev)
{
	ledSetState(_dev, Off);
}    


bool ledIsOn(ledDev_st *_dev)
{
    return (_dev->state == On ? true : false);
}


bool ledIsOff(ledDev_st *_dev)
{
    return !ledIsOn(_dev);
}


void ledReset(ledDev_st *_dev)
{
	ledSetState(_dev, _dev->initialState);
}    


void ledToggle(ledDev_st *_dev)
{
	ledSetState(_dev, _dev->state == On ? Off : On);
}


void ledBlink(ledDev_st *_dev,
    uint8_t count,
    uint16_t onTimeMs,
    uint16_t offTimeMs,
    uint16_t leadOffTimeMs,
    uint16_t trailOffTimeMs)
{
    // Simple flash implementation.
    // Synchronous, caller has to wait for completion.
    // leadOffTimeMs and trailOffTimeMs are only
    // used if LED is currently on.
    
	ledState_e savedState = _dev->state;

    if (savedState == On)
    {
    	ledSetState(_dev, Off);
        HAL_Delay(leadOffTimeMs);
    }

    for (uint8_t flash = 0; flash < count; ++flash)
    {
    	ledSetState(_dev, On);
    	HAL_Delay(onTimeMs);

        if (flash < count - 1)
        {
        	ledSetState(_dev, Off);
        	HAL_Delay(offTimeMs);
        }
    }

    if (savedState == On)
    {
    	ledSetState(_dev, Off);
    	HAL_Delay(trailOffTimeMs);
    }
    ledSetState(_dev, savedState);
}




void ledForceState(ledDev_st *_dev, ledState_e state)
{
    if (state == On){
    	HAL_GPIO_WritePin(_dev->port, _dev->pin, _dev->activeLevel);
    }
    else{
    	HAL_GPIO_WritePin(_dev->port, _dev->pin, _dev->activeLevel ^ 1);
    }
    _dev->state = state;
}





