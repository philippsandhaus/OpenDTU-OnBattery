// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <gridcharger/Stats.h>
#include <mutex>
#include <optional>

namespace GridChargers::Shelly {

class Stats : public ::GridChargers::Stats {
friend class Provider;

public:
    uint32_t getLastUpdate() const final;
    std::optional<float> getInputPower() const final;
    void getLiveViewData(JsonVariant& root) const final;

protected:
    void mqttPublish() const final {}

private:
    mutable std::mutex _mutex;
    uint32_t _lastUpdate = 0;
    bool _isOn = false;
    std::optional<float> _measuredPowerW;  // from Shelly power metering, if available
    float _ratedPowerW = 0.0f;             // FixedAmperage * 230V
};

} // namespace GridChargers::Shelly
