// SPDX-License-Identifier: GPL-2.0-or-later

#include <gridcharger/victron/Stats.h>
#include <Configuration.h>
#include <MqttSettings.h>
#include <LogHelper.h>

#undef TAG
static const char* TAG = "gridCharger";
static const char* SUBTAG = "VictronIP22";

namespace GridChargers::Victron {

void Stats::updateFrom(veChargerStruct const& data, uint32_t lastUpdate)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _lastUpdate = lastUpdate;
    _batteryVoltage_mV = data.batteryVoltage_V_mV;
    _batteryCurrent_mA = data.batteryCurrent_I_mA;
    _outputPower_W     = data.outputPower_W;
    _currentState_CS   = data.currentState_CS;
    _errorCode_ERR     = data.errorCode_ERR;
    _offReason_OR      = data.offReason_OR;
    _productID         = data.productID_PID;
    strncpy(_serialNr,    data.serialNr_SER,    sizeof(_serialNr) - 1);
    strncpy(_firmwareVer, data.firmwareVer_FW[0] != '\0' ? data.firmwareVer_FW : data.firmwareVer_FWE,
            sizeof(_firmwareVer) - 1);

    if (data.ChargerVoltageMilliVolt.first > 0) {
        _chargerVoltage_V = data.ChargerVoltageMilliVolt.second / 1000.0f;
    } else {
        _chargerVoltage_V.reset();
    }

    if (data.ChargerCurrentMilliAmp.first > 0) {
        _chargerCurrent_A = data.ChargerCurrentMilliAmp.second / 1000.0f;
    } else {
        _chargerCurrent_A.reset();
    }
}

uint32_t Stats::getLastUpdate() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _lastUpdate;
}

std::optional<float> Stats::getInputPower() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_outputPower_W > 0) {
        return static_cast<float>(_outputPower_W);
    }
    return 0.0f;
}

void Stats::getLiveViewData(JsonVariant& root) const
{
    root["vendorName"] = "Victron Energy";
    root["productName"] = "Blue Smart IP22 Charger";
    root["provider"] = GridChargerProviderType::VICTRON_IP22;

    std::lock_guard<std::mutex> lock(_mutex);

    const auto dataAge = millis() - _lastUpdate;
    root["dataAge"] = dataAge;
    root["reachable"] = _lastUpdate > 0 && dataAge < 30000;

    bool producing = _currentState_CS >= static_cast<uint8_t>(veChargerState::Bulk)
                  && _currentState_CS != static_cast<uint8_t>(veChargerState::Off)
                  && _currentState_CS != static_cast<uint8_t>(veChargerState::Fault);
    root["producing"] = producing;

    veChargerStruct dummy;
    dummy.currentState_CS = _currentState_CS;
    dummy.errorCode_ERR   = _errorCode_ERR;
    dummy.offReason_OR    = _offReason_OR;
    addStringInSection(root, "device", "status",   std::string(dummy.getCsAsString().data()),  false);
    addStringInSection(root, "device", "error",    std::string(dummy.getErrAsString().data()),  false);
    addStringInSection(root, "device", "offReason", std::string(dummy.getOrAsString().data()), false);
    addStringInSection(root, "device", "serial",   std::string(_serialNr), false);
    addStringInSection(root, "device", "firmware", std::string(_firmwareVer), false);

    addValueInSection(root, "output", "voltage",  _batteryVoltage_mV / 1000.0f, "V", 2);
    addValueInSection(root, "output", "current",  _batteryCurrent_mA / 1000.0f, "A", 2);
    addValueInSection(root, "output", "power",    static_cast<float>(_outputPower_W), "W", 0);

    if (_chargerVoltage_V.has_value()) {
        addValueInSection(root, "output", "chargerVoltage", *_chargerVoltage_V, "V", 2);
    }
    if (_chargerCurrent_A.has_value()) {
        addValueInSection(root, "output", "chargerCurrent", *_chargerCurrent_A, "A", 2);
    }
}

void Stats::mqttPublish() const
{
    auto& mqttTopic = Configuration.get().Mqtt.Topic;

    std::string prefix = std::string(mqttTopic) + "gridcharger/victron/";

    std::lock_guard<std::mutex> lock(_mutex);

    MqttSettings.publish((prefix + "battery_voltage").c_str(),
            String(_batteryVoltage_mV / 1000.0f, 2).c_str());
    MqttSettings.publish((prefix + "battery_current").c_str(),
            String(_batteryCurrent_mA / 1000.0f, 2).c_str());
    MqttSettings.publish((prefix + "output_power").c_str(),
            String(_outputPower_W).c_str());
    MqttSettings.publish((prefix + "charge_state").c_str(),
            String(_currentState_CS).c_str());

    if (_chargerVoltage_V.has_value()) {
        MqttSettings.publish((prefix + "charger_voltage").c_str(),
                String(*_chargerVoltage_V, 2).c_str());
    }
    if (_chargerCurrent_A.has_value()) {
        MqttSettings.publish((prefix + "charger_current").c_str(),
                String(*_chargerCurrent_A, 2).c_str());
    }
}

} // namespace GridChargers::Victron
