/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. 
   
   Additional code by Chuck Swiger 2019  */

// This sample C application for Azure Sphere demonstrates GPS using Mikroe Nano-GPS Click Module
// https://www.mikroe.com/nano-gps-click
// https://download.mikroe.com/documents/add-on-boards/click/nano-gps/nano-gps%20click-manual-v100.pdf
// https://origingps.com/wp-content/uploads/2018/12/Nano-Hornet-ORG1411-Datasheet-Rev-4.1.pdf
// Using GPIO to control the device power state and UART at 4800 baud to read NMEA sentences
// 

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/uart.h>
#include <applibs/gpio.h>
#include <applibs/log.h>

// This sample is targeted at the Avnet MT3620 avnet_mt3620_sk
// This can be changed using the project property "Target Hardware Definition Directory".
// This #include imports the sample_hardware abstraction from that hardware definition.
#include <hw/sample_hardware.h>

// This sample uses a single-thread event loop pattern, based on epoll and timerfd
#include "epoll_timerfd_utilities.h"

// File descriptors - initialized to invalid value
static int gpsPwrGpioFd = -1;		//  AVNET_MT3620_SK_GPIO0 on Click Socket1 PWM to board PWR ON_OFF input line
static int gpsWakeupGpioFd = -1;    //   AVNET_MT3620_SK_GPIO42 on Click Socket1 AN to board WAKEUP
static int SampleBlueLedGpioFd = -1;    // On board BLUE LED  SAMPLE_RGBLED_BLUE
static int uartFd = -1;				// UART ISU0 TX/RX on both sockets
static int gpsInitTimerFd = -1;	
static int epollFd = -1;


// Pulse interval variables - this struct is { time_t tv_sec; long tv_nsec; }, Seconds and NanoSeconds
// will need to pulse the GPS PWM PWR / ON_OFF pad for 100mSec per 
// https://origingps.com/wp-content/uploads/2018/12/Nano-Hornet-ORG1411-Datasheet-Rev-4.1.pdf   p.27 $21. Operation
// Using 500uSec
static const struct timespec pulseInterval = {0, 500000};

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///     Handle Pulse PWR timer event - somehow.
/// </summary>
static void gpsInitTimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(gpsInitTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

	// The gps PWR interval has elapsed, so just turn off the PWR pin and timer
	// The PIN is assumed normal GPIO_Value_Low is off (0) and GPIO_Value_High is on (1v8 at the GPS module)
	// The may not have been set High if the AWAKE line was already on after a reset
	int result = GPIO_SetValue(gpsPwrGpioFd, GPIO_Value_Low);
	if (result != 0) {
		Log_Debug("ERROR: Could not set GPS PWR output value: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
	}

	// Just assume it works and turn off the timer for wakup pulse
	// ideal would be to actually check the WAKEUP pin and repeat 100mSec pulses every second until true
	result = UnregisterEventHandlerFromEpoll(epollFd, gpsInitTimerFd);

	// Check for WAKEUP
	GPIO_Value_Type gpsWakeupState;
	result = GPIO_GetValue(gpsWakeupGpioFd, &gpsWakeupState);
	if (result != 0) {
		Log_Debug("ERROR: Could not read GPS WAKEUP: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}
	if (gpsWakeupState == GPIO_Value_High) {
		Log_Debug("GPS Awake\n");
		// If GPS unit is already or now AWAKE turn on the Blue LED. For LEDs Low is active ON
		result = GPIO_SetValue(SampleBlueLedGpioFd, GPIO_Value_Low);
		if (result != 0) {
			Log_Debug("ERROR: Could not set Blue LED output value: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}
	}

	Log_Debug("Now GPS data from UART\n");

}

/// <summary>
///     Handle UART event: if there is incoming data, print it.
/// </summary>
static void UartEventHandler(EventData* eventData)
{
	const size_t receiveBufferSize = 256;
	uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
	ssize_t bytesRead;

	// Read incoming UART data. It is expected behavior that messages may be received in multiple
	// partial chunks.
	bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
	if (bytesRead < 0) {
		Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (bytesRead > 0) {
		// Null terminate the buffer to make it a valid string, and print it
		receiveBuffer[bytesRead] = 0;
		// \r so debug out does not barberpole
		Log_Debug("UART received %d bytes: '%s'.\n\r", bytesRead, (char*)receiveBuffer);
	}
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData gpsInitTimerEventData = {.eventHandler = &gpsInitTimerEventHandler};
static EventData uartEventData = { .eventHandler = &UartEventHandler };

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0) {
		return -1;
	}


	// Open PWR GPIO, set as output with value GPIO_Value_Low (off)
	Log_Debug("Opening GPS PWR as output.\n");
	gpsPwrGpioFd = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO0, GPIO_OutputMode_PushPull, GPIO_Value_Low);
	if (gpsPwrGpioFd < 0) {
		Log_Debug("ERROR: Could not open GPS PWR GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	Log_Debug("Opening GPS WAKEUP as input.\n");
	gpsWakeupGpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_GPIO42);	// input from GPS WAKEUP pad
	if (gpsPwrGpioFd < 0) {
		Log_Debug("ERROR: Could not open GPS WAKUP GPIO: %s (%d).\n,strerror(errno), errno");
		return -1;
	}

	// Open BLUE LED GPIO, set as output with value GPIO_Value_High (led off)
	Log_Debug("Opening LED as output.\n");
	SampleBlueLedGpioFd = GPIO_OpenAsOutput(SAMPLE_RGBLED_BLUE, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (SampleBlueLedGpioFd < 0) {
		Log_Debug("ERROR: Could not open Blue LED GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return -1;
	}

	// Now check WAKEUP and only send a pulse if OFF
	GPIO_Value_Type gpsWakeupState;
	int result = GPIO_GetValue(gpsWakeupGpioFd, &gpsWakeupState);
	if (result != 0) {
		Log_Debug("ERROR: Could not read GPS WAKEUP: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return -1;
	}
	if (gpsWakeupState == GPIO_Value_High) {
		Log_Debug("GPS already awake\n");
	}
	else {
		result = GPIO_SetValue(gpsPwrGpioFd, GPIO_Value_High);
		if (result != 0) {
			Log_Debug("ERROR: Could not set GPS PWR output value: %s (%d).\n", strerror(errno), errno);
			terminationRequired = true;
			return -1;
		}
	}


	gpsInitTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &pulseInterval,
                                                    &gpsInitTimerEventData, EPOLLIN);
	if (gpsInitTimerFd < 0) {
		return -1;
	}


	// Create a UART_Config object, open the UART and set up UART event handler
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 4800;
	uartConfig.flowControl = UART_FlowControl_None;
	uartFd = UART_Open(SAMPLE_UART, &uartConfig);
	if (uartFd < 0) {
		Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	if (RegisterEventHandlerToEpoll(epollFd, uartFd, &uartEventData, EPOLLIN) != 0) {
	return -1;
	}

	// everything worked, return zero status
	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    // Leave the GPS PWR off
    if (gpsPwrGpioFd >= 0) {
        GPIO_SetValue(gpsPwrGpioFd, GPIO_Value_Low);
    }

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(gpsInitTimerFd, "BlinkingLedTimer");
    CloseFdAndPrintError(gpsPwrGpioFd, "BlinkingLedGpio");
    CloseFdAndPrintError(epollFd, "Epoll");
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("GPS Init application starting.\n");
    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}