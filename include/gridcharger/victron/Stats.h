// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <gridcharger/Stats.h>
#include <VeDirectData.h>
#include <mutex>
#include <optional>

namespace GridChargers::Victron {

class Stats : public ::GridChargers::Stats {
    friend class Provider;

public:
    uint32_t getLastUpdate() const final;
    std::optional<float> getInputPower() const final;
    void getLiveViewData(JsonVariant& root) const final;

protected:
    void mqttPublish() const final;

private:
    void updateFrom(veChargerStruct const& data, uint32_t lastUpdate);

    mutable std::mutex _mutex;
    uint32_t _lastUpdate = 0;

    // mirrored fields from veChargerStruct
    uint32_t _batteryVoltage_mV = 0;
    int32_t  _batteryCurrent_mA = 0;
    int16_t  _outputPower_W = 0;
    uint8_t  _currentState_CS = 0;
    uint8_t  _errorCode_ERR = 0;
    uint32_t _offReason_OR = 0;
    uint16_t _productID = 0;
    char     _serialNr[VE_MAX_VALUE_LEN] = {};
    char     _firmwareVer[VE_MAX_VALUE_LEN] = {};

    // HEX values
    std::optional<float> _chargerVoltage_V;
    std::optional<float> _chargerCurrent_A;
};

} // namespace GridChargers::Victron
