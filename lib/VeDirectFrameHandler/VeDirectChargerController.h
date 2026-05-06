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

    // Set the charge current limit in Ampere (rounded to the nearest integer).
    // Uses register 0xEDF0 "Battery maximum current" (1 A/unit, uint16).
    // Source: BlueSolar-HEX-protocol.pdf.
    // IMPORTANT: 0xEDF0 is in the EEPROM-mapped range — writes are rate-limited
    // internally to at most once per MIN_CURRENT_WRITE_INTERVAL_MS to protect flash.
    void setChargeCurrent(float ampere);

    // Enable remote on/off control (sets bit 1 in register 0x0202).
    // Must be called once after power-up before DeviceMode commands are accepted.
    // Source: BlueSolar-HEX-protocol.pdf, register 0x0202, Note 1.
    void enableRemoteControl();

    // Turn the charger on (DeviceMode = 1) or off (DeviceMode = 4).
    // Only effective after enableRemoteControl() has been called.
    void setDeviceMode(bool on);

private:
    bool hexDataHandler(VeDirectHexData const& data) final;
    bool processTextDataDerived(std::string const& name, std::string const& value) final;
    void frameValidEvent() final;

    void sendNextHexCommandFromQueue();
    bool isHexCommandPossible() const;

    uint32_t _sendTimeout = 0;
    size_t   _sendQueueNr = 0;

    static constexpr uint32_t MIN_CURRENT_WRITE_INTERVAL_MS = 60000; // protect EEPROM
    uint32_t _lastCurrentWriteMillis = 0;
    float    _lastWrittenCurrentA    = -1.0f;

    #define CHARGER_HIGH_PRIO_CMD 1
    std::array<VeDirectChargerHexQueue, 4> _hexQueue {{
        // Poll charger voltage and current at high priority
        { VeDirectHexRegister::ChargerVoltage,  false, CHARGER_HIGH_PRIO_CMD, 0, 0, std::nullopt },
        { VeDirectHexRegister::ChargerCurrent,  false, CHARGER_HIGH_PRIO_CMD, 0, 0, std::nullopt },
        // Poll device state every 4 s
        { VeDirectHexRegister::DeviceState,     false, 4,                    0, 0, std::nullopt },
        // Battery max current (0xEDF0) — SET command, queued on demand, rate-limited
        { VeDirectHexRegister::BatteryMaxCurrent, true, 0,                   0, 16, std::nullopt },
    }};
};
