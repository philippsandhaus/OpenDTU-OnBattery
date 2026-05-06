// SPDX-License-Identifier: GPL-2.0-or-later

#include <gridcharger/victron/Provider.h>
#include <battery/Controller.h>
#include <powermeter/Controller.h>
#include <PowerLimiter.h>
#include <Configuration.h>
#include <PinMapping.h>
#include <SerialPortManager.h>
#include <LogHelper.h>

#undef TAG
static const char* TAG = "gridCharger";
static const char* SUBTAG = "VictronIP22";

namespace GridChargers::Victron {

bool Provider::init()
{
    DTU_LOGI("Initialize Victron Blue Smart IP22 grid charger...");

    const PinMapping_t& pin = PinMapping.get();

    if (pin.victron_charger_rx <= GPIO_NUM_NC) {
        DTU_LOGE("RX pin not configured in pin mapping (victron_charger_rx)");
        return false;
    }

    _serialPortOwner = "Victron Charger";
    auto oPort = SerialPortManager.allocatePort(_serialPortOwner.c_str());
    if (!oPort) {
        DTU_LOGE("Failed to allocate serial port");
        return false;
    }

    _controller = std::make_unique<VeDirectChargerController>();
    _controller->init(pin.victron_charger_rx, pin.victron_charger_tx, *oPort);

    DTU_LOGI("Initialized on rx=%d tx=%d", pin.victron_charger_rx, pin.victron_charger_tx);
    _remoteControlEnabled = false;
    return true;
}

void Provider::deinit()
{
    _controller.reset();
    if (!_serialPortOwner.empty()) {
        SerialPortManager.freePort(_serialPortOwner.c_str());
        _serialPortOwner.clear();
    }
}

void Provider::loop()
{
    if (!_controller) { return; }

    _controller->loop();

    // Enable remote on/off control once firmware version is known (FW >= 1.53).
    if (!_remoteControlEnabled && _controller->isDataValid()) {
        _controller->enableRemoteControl();
        _remoteControlEnabled = true;
    }

    if (_controller->isDataValid()) {
        _stats->updateFrom(_controller->getData(), _controller->getLastUpdate());

        auto const& data = _controller->getData();
        _isCharging = data.currentState_CS >= static_cast<uint8_t>(veChargerState::Bulk)
                   && data.currentState_CS != static_cast<uint8_t>(veChargerState::Off)
                   && data.currentState_CS != static_cast<uint8_t>(veChargerState::Fault);
    }

    powerControlLoop();
}

void Provider::powerControlLoop()
{
    if (!_controller->isDataValid()) { return; }

    if (millis() - _lastControlMillis < CONTROL_INTERVAL_MS) { return; }
    _lastControlMillis = millis();

    auto const& config = Configuration.get();
    auto const& data   = _controller->getData();

    float batteryVoltage_V = data.batteryVoltage_V_mV / 1000.0f;
    if (batteryVoltage_V < 1.0f) {
        DTU_LOGW("Battery voltage invalid (%.2f V), skipping power control", batteryVoltage_V);
        return;
    }

    // ── Emergency charging ──────────────────────────────────────────────────
    auto batteryStats = Battery.getStats();
    if (!_batteryEmergencyCharging
            && config.GridCharger.EmergencyChargeEnabled
            && batteryStats->getImmediateChargingRequest()) {
        _batteryEmergencyCharging = true;
        DTU_LOGI("Emergency charging: applying max current %.1f A",
                config.GridCharger.Victron.MaxCurrentA);
        applyCurrentSetpoint(config.GridCharger.Victron.MaxCurrentA);
        return;
    }

    if (_batteryEmergencyCharging && !batteryStats->getImmediateChargingRequest()) {
        applyCurrentSetpoint(0.0f);
        _batteryEmergencyCharging = false;
        return;
    }

    if (_batteryEmergencyCharging) { return; }

    // ── Auto power control ───────────────────────────────────────────────────
    if (!config.GridCharger.AutoPowerEnabled) { return; }

    if (_autoModeBlockedTillMillis > millis()) { return; }

    if (PowerLimiter.isGovernedBatteryPoweredInverterProducing()) {
        applyCurrentSetpoint(0.0f);
        _autoPowerEnabled = false;
        DTU_LOGI("Inverter active — disabling charger");
        _autoModeBlockedTillMillis = millis() + 5000;
        return;
    }

    if (!PowerMeter.isDataValid()) {
        DTU_LOGW("Power meter data invalid, skipping auto control");
        _autoModeBlockedTillMillis = millis() + 1000;
        return;
    }

    if (PowerMeter.getLastUpdate() <= _lastPowerMeterUpdateMillis) { return; }
    _lastPowerMeterUpdateMillis = PowerMeter.getLastUpdate();

    // SoC limit check
    if (config.Battery.Enabled && config.GridCharger.AutoPowerBatterySoCLimitsEnabled) {
        if (batteryStats->getSoC() >= config.GridCharger.AutoPowerStopBatterySoCThreshold) {
            applyCurrentSetpoint(0.0f);
            _autoPowerEnabled = false;
            DTU_LOGV("SoC limit reached, disabling charger");
            return;
        }
    }

    _autoPowerEnabled = true;

    float gridPower_W    = PowerMeter.getPowerTotal();
    float targetDelta_W  = config.GridCharger.AutoPowerTargetPowerConsumption;
    float chargePower_W  = -gridPower_W + static_cast<float>(data.outputPower_W) + targetDelta_W;

    // Clamp to [0, max]
    float maxPower_W = config.GridCharger.Victron.MaxCurrentA * batteryVoltage_V;
    chargePower_W = std::max(0.0f, std::min(chargePower_W, maxPower_W));

    // Respect BMS charge current limit
    float bmsCurrent_A = batteryStats->getChargeCurrentLimit()
                       - (batteryStats->getChargeCurrent() - data.batteryCurrent_I_mA / 1000.0f);
    float current_A = std::min(chargePower_W / batteryVoltage_V, bmsCurrent_A);
    current_A = std::max(0.0f, std::min(current_A, config.GridCharger.Victron.MaxCurrentA));

    DTU_LOGV("gridPower=%.1fW chargePower=%.1fW current=%.2fA", gridPower_W, chargePower_W, current_A);

    applyCurrentSetpoint(current_A);
    _autoModeBlockedTillMillis = millis() + CONTROL_INTERVAL_MS * 2;
}

void Provider::applyCurrentSetpoint(float ampere)
{
    if (!_controller) { return; }

    // Only send if the setpoint changed more than the hysteresis threshold
    if (std::abs(ampere - _lastSetpointA) < SETPOINT_HYSTERESIS_A) { return; }

    _lastSetpointA = ampere;

    if (ampere <= 0.0f) {
        _controller->setDeviceMode(false);
        DTU_LOGI("Turning charger off");
    } else {
        _controller->setDeviceMode(true);
        _controller->setChargeCurrent(ampere);
        DTU_LOGI("Setting charge current to %.2f A", ampere);
    }
}

} // namespace GridChargers::Victron
