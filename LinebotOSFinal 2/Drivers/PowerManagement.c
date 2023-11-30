/*
 * PowerManagement.c
 *
 * Created: 7/11/2023 14:36:16
 *  Author: maxco
 */ 
#include "PowerManagement.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "DriverPower.h"

void enterSleepMode(void) {
	// Set the sleep mode. For XMEGA, you might use SLEEP_SMODE_PDOWN_gc for power down.
	// Check the specific group configuration (gc) value for your microcontroller.
	set_sleep_mode(SLEEP_SMODE_PDOWN_gc);

	// Disable any peripherals here to save power.
	// For example, power_adc_disable(); (if such function exists or equivalent)

	// Enable global interrupts so that the MCU can wake up from an interrupt.
	sei();
	
	vTaskDelay(100);
	// Enable sleep.
	
	
	PMIC.CTRL &=0b11111100; // shut off medium and low lever interrupt
	DriverPowerVccAuxSet(0);
	sleep_enable();

	vTaskDelay(50);
	// Enter sleep mode.
	sleep_cpu();

	// Sleep mode will be exited upon receiving an interrupt.

	// Disable sleep after waking up.
	sleep_disable();

	// Re-enable peripherals if needed.
	// For example, power_adc_enable(); (if such function exists or equivalent)
}

