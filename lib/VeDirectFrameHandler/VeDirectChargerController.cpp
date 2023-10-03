#include <Arduino.h>
#include "VeDirectChargerController.h"

/*
 * setBatteryCurrentLimit
 * This function sets battery current limit. Don't call it in a loop because 
 * Victron is writing battery current limit in non volatile memory.
 */
void VeDirectChargerController::setBatteryCurrentLimit(uint16_t batteryCurrentLimit)
{
	//uint16_t veBatCurrentLimit = batteryCurrentLimit * 10;
	sendHexCommand(SET,BATTERY_MAXIMUM_CURRENT,DEFAULT_FLAG0, batteryCurrentLimit * 10);
	_msgOut->printf("[Victron Charger] Set Victron batteryMaximumCurrent to %dA.\r\n",batteryCurrentLimit);
}
