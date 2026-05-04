// SPDX-License-Identifier: GPL-2.0-or-later

#include <Arduino.h>
#include <gridcharger/shelly/Stats.h>
#include <Configuration.h>

namespace GridChargers::Shelly {

uint32_t Stats::getLastUpdate() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _lastUpdate;
}

std::optional<float> Stats::getInputPower() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_measuredPowerW.has_value()) {
        return _measuredPowerW;
    }
    if (_isOn) {
        return _ratedPowerW;
    }
    return 0.0f;
}

void Stats::getLiveViewData(JsonVariant& root) const
{
    root["vendorName"] = "Shelly";
    root["productName"] = "Smart Switch Charger";
    root["provider"] = GridChargerProviderType::SHELLY;

    std::lock_guard<std::mutex> lock(_mutex);

    const auto dataAge = millis() - _lastUpdate;
    root["dataAge"] = dataAge;
    root["reachable"] = _lastUpdate > 0 && dataAge < 30000;
    root["producing"] = _isOn;

    addStringInSection(root, "device", "state", _isOn ? std::string("shelly.on") : std::string("shelly.off"));

    if (_measuredPowerW.has_value()) {
        addValueInSection(root, "input", "power", *_measuredPowerW, "W", 1);
    } else if (_isOn) {
        addValueInSection(root, "input", "power", _ratedPowerW, "W", 1);
    }
    addValueInSection(root, "settings", "ratedPower", _ratedPowerW, "W", 0);
}

} // namespace GridChargers::Shelly
