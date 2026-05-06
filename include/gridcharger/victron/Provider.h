// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <gridcharger/Provider.h>
#include <gridcharger/victron/Stats.h>
#include <VeDirectChargerController.h>
#include <memory>
#include <mutex>

namespace GridChargers::Victron {

class Provider : public ::GridChargers::Provider {
public:
    bool init() final;
    void deinit() final;
    void loop() final;

    std::shared_ptr<::GridChargers::Stats> getStats() const final { return _stats; }

    bool getAutoPowerStatus() const final { return _isCharging; }

private:
    void powerControlLoop();
    void applyCurrentSetpoint(float ampere);

    std::unique_ptr<VeDirectChargerController> _controller;
    std::string _serialPortOwner;

    std::shared_ptr<Stats> _stats = std::make_shared<Stats>();

    float _lastSetpointA = -1.0f;
    bool  _isCharging = false;
    bool  _batteryEmergencyCharging = false;
    bool  _remoteControlEnabled = false;

    uint32_t _lastPowerMeterUpdateMillis = 0;
    uint32_t _autoModeBlockedTillMillis  = 0;
    bool     _autoPowerEnabled = false;

    static constexpr float SETPOINT_HYSTERESIS_A = 0.1f;
    static constexpr int   CONTROL_INTERVAL_MS   = 2000;
    uint32_t _lastControlMillis = 0;
};

} // namespace GridChargers::Victron
