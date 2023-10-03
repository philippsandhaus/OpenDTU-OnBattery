#pragma once

#include <Arduino.h>
#include "VeDirectFrameHandler.h"

class VeDirectChargerController : public VeDirectFrameHandler {
public:
    void setBatteryCurrentLimit(uint16_t batteryCurrentLimit);
};