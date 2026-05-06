// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <Arduino.h>
#include <optional>
#include "VeDirectData.h"
#include "VeDirectFrameHandler.h"

struct VeDirectChargerHexQueue {
    VeDirectHexRegister _hexRegister;
    bool _setCommand;
    uint8_t _readPeriod;        // seconds between GET polls (0 = high-prio, sent every loop)
    uint32_t _lastSendTime;
    uint8_t _dataLength;        // bit-width for SET (8/16/32)
    std::optional<uint32_t> _data;
};

class VeDirectChargerController : public VeDirectFrameHandler<veChargerStruct> {
public:
    VeDirectChargerController() = default;

    void init(gpio_num_t rx, gpio_num_t tx, uint8_t hwSerialPort);

    using data_t = veChargerStruct;

    void loop() final;

    // Set the charge current limit in Ampere.
    // Converted to register units (0.1 A/unit) and queued for the next HEX send slot.
    // NOTE: register 0x2015 is a volatile RAM register — safe for control-loop writes.
    // Verify register address and unit scale against the VE.Direct Blue Smart charger
    // HEX protocol documentation before deploying.
    void setChargeCurrent(float ampere);

    // Turn the charger on (DeviceMode = 1) or off (DeviceMode = 4).
    void setDeviceMode(bool on);

private:
    bool hexDataHandler(VeDirectHexData const& data) final;
    bool processTextDataDerived(std::string const& name, std::string const& value) final;
    void frameValidEvent() final;

    void sendNextHexCommandFromQueue();
    bool isHexCommandPossible() const;

    uint32_t _sendTimeout = 0;
    size_t   _sendQueueNr = 0;

    #define CHARGER_HIGH_PRIO_CMD 1
    std::array<VeDirectChargerHexQueue, 4> _hexQueue {{
        // Poll charger voltage and current at high priority
        { VeDirectHexRegister::ChargerVoltage,    false, CHARGER_HIGH_PRIO_CMD, 0, 0, std::nullopt },
        { VeDirectHexRegister::ChargerCurrent,    false, CHARGER_HIGH_PRIO_CMD, 0, 0, std::nullopt },
        // Poll device state every 4 s
        { VeDirectHexRegister::DeviceState,       false, 4,                    0, 0, std::nullopt },
        // Charge current limit — SET command, queued on demand
        { VeDirectHexRegister::ChargeCurrentLimit, true, 0,                    0, 16, std::nullopt },
    }};
};
