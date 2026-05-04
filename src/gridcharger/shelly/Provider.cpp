// SPDX-License-Identifier: GPL-2.0-or-later

#include <gridcharger/shelly/Provider.h>
#include <battery/Controller.h>
#include <powermeter/Controller.h>
#include <PowerLimiter.h>
#include <Configuration.h>
#include <LogHelper.h>
#include <HTTPClient.h>

#undef TAG
static const char* TAG = "gridCharger";
static const char* SUBTAG = "Shelly";

namespace GridChargers::Shelly {

bool Provider::init()
{
    DTU_LOGI("Initialize Shelly grid charger interface...");

    auto const& config = Configuration.get();
    auto const& ipAddress = IPAddress(config.GridCharger.Shelly.IpAddress);

    if (ipAddress.toString() == "0.0.0.0") {
        DTU_LOGE("Invalid IP address: %s", ipAddress.toString().c_str());
        return false;
    }

    _controlBaseUrl = "http://" + ipAddress.toString();

    _statusRequestConfig = std::make_unique<HttpRequestConfig>();
    strlcpy(_statusRequestConfig->Url,
            (_controlBaseUrl + "/status").c_str(),
            sizeof(_statusRequestConfig->Url));
    _statusRequestConfig->Timeout = HTTP_TIMEOUT_MS;
    _statusRequestConfig->AuthType = HttpRequestConfig::Auth::None;
    strlcpy(_statusRequestConfig->HeaderKey, "", sizeof(_statusRequestConfig->HeaderKey));
    strlcpy(_statusRequestConfig->HeaderValue, "", sizeof(_statusRequestConfig->HeaderValue));
    strlcpy(_statusRequestConfig->Username, "", sizeof(_statusRequestConfig->Username));
    strlcpy(_statusRequestConfig->Password, "", sizeof(_statusRequestConfig->Password));

    _statusGetter = std::make_unique<HttpGetter>(*_statusRequestConfig);
    _statusGetter->addHeader("Accept", "application/json");

    if (!_statusGetter->init()) {
        DTU_LOGE("Initializing HTTP getter failed: %s", _statusGetter->getErrorText());
        _statusGetter = nullptr;
        return false;
    }

    // Initialise rated power in stats
    {
        std::lock_guard<std::mutex> lock(_stats->_mutex);
        _stats->_ratedPowerW = config.GridCharger.Shelly.FixedAmperage * AC_VOLTAGE;
    }

    DTU_LOGI("Shelly charger ready at %s (rated %.1f W)",
             ipAddress.toString().c_str(),
             config.GridCharger.Shelly.FixedAmperage * AC_VOLTAGE);

    return true;
}

void Provider::deinit()
{
    _pollingTaskDone = false;

    {
        std::unique_lock<std::mutex> lock(_pollingMutex);
        _stopPolling = true;
    }
    _pollingCv.notify_all();

    if (_pollingTaskHandle != nullptr) {
        while (!_pollingTaskDone) { delay(10); }
        _pollingTaskHandle = nullptr;
    }

    // Turn off the charger on deinit
    if (_isOn) {
        sendSwitchCommand(false);
    }

    _statusGetter = nullptr;
    _statusRequestConfig = nullptr;
}

void Provider::loop()
{
    powerControlLoop();

    if (_pollingTaskHandle == nullptr) {
        std::unique_lock<std::mutex> lock(_pollingMutex);
        _stopPolling = false;
        lock.unlock();

        uint32_t constexpr stackSize = 4096;
        xTaskCreate(pollingLoopHelper, "ShellyPolling",
                stackSize, this, 1/*prio*/, &_pollingTaskHandle);
    }

    // Apply target switch state if changed and enough time has passed
    if (_targetState != _isOn) {
        if (millis() - _lastSwitchMillis >= MIN_SWITCH_INTERVAL_MS || _lastSwitchMillis == 0) {
            sendSwitchCommand(_targetState);
        }
    }
}

void Provider::powerControlLoop()
{
    auto const& config = Configuration.get();

    // ***********************
    // Emergency charge
    // ***********************
    auto stats = Battery.getStats();
    if (!_batteryEmergencyCharging &&
        config.GridCharger.EmergencyChargeEnabled &&
        stats->getImmediateChargingRequest()) {
        _batteryEmergencyCharging = true;
        _targetState = true;
        DTU_LOGI("Emergency charge: turning Shelly ON");
        return;
    }

    if (_batteryEmergencyCharging && !stats->getImmediateChargingRequest()) {
        _batteryEmergencyCharging = false;
        _targetState = false;
        DTU_LOGI("Emergency charge ended: turning Shelly OFF");
        return;
    }

    if (_batteryEmergencyCharging) { return; }

    // ***********************
    // Automatic power control
    // ***********************
    if (!config.GridCharger.AutoPowerEnabled) { return; }

    if (PowerLimiter.isGovernedBatteryPoweredInverterProducing()) {
        if (_targetState) {
            DTU_LOGI("Inverter is active, disabling Shelly charger");
            _targetState = false;
        }
        return;
    }

    // Check battery SoC limit
    if (config.Battery.Enabled && config.GridCharger.AutoPowerBatterySoCLimitsEnabled) {
        uint8_t soc = Battery.getStats()->getSoC();
        if (soc >= config.GridCharger.AutoPowerStopBatterySoCThreshold) {
            if (_targetState) {
                DTU_LOGD("Battery SoC %u%% reached stop threshold, turning Shelly OFF", soc);
                _targetState = false;
            }
            return;
        }
    }

    // Only recalculate at the control interval to avoid thrashing
    if (millis() - _lastControlMillis < CONTROL_INTERVAL_MS) { return; }
    _lastControlMillis = millis();

    float const ratedPowerW = config.GridCharger.Shelly.FixedAmperage * AC_VOLTAGE;
    float const powerTotal = PowerMeter.getPowerTotal();
    float const target = config.GridCharger.AutoPowerTargetPowerConsumption;

    // Turn ON when: adding the charger load keeps us at or below the target consumption.
    // powerTotal + ratedPowerW <= target
    // → we have enough surplus (or target headroom) to run the charger
    bool const shouldBeOn = (powerTotal + ratedPowerW) <= target;

    if (shouldBeOn != _targetState) {
        DTU_LOGD("Auto-power: powerTotal=%.0fW, rated=%.0fW, target=%.0fW → %s",
                 powerTotal, ratedPowerW, target, shouldBeOn ? "ON" : "OFF");
        _targetState = shouldBeOn;
    }
}

void Provider::sendSwitchCommand(bool on)
{
    if (!WiFi.isConnected()) { return; }

    String url = _controlBaseUrl + "/relay/0?turn=" + (on ? "on" : "off");
    HTTPClient http;
    http.begin(url);
    http.setTimeout(HTTP_TIMEOUT_MS);
    int code = http.GET();

    if (code > 0) {
        _isOn = on;
        _lastSwitchMillis = millis();
        DTU_LOGI("Shelly switch %s (HTTP %d)", on ? "ON" : "OFF", code);
    } else {
        DTU_LOGE("Shelly switch command failed: %s", http.errorToString(code).c_str());
    }

    http.end();

    // Update stats
    {
        std::lock_guard<std::mutex> lock(_stats->_mutex);
        _stats->_isOn = _isOn;
        _stats->_lastUpdate = millis();
    }
}

void Provider::pollingLoopHelper(void* context)
{
    auto* pInstance = static_cast<Provider*>(context);
    pInstance->pollingLoop();
    pInstance->_pollingTaskDone = true;
    vTaskDelete(nullptr);
}

void Provider::pollingLoop()
{
    std::unique_lock<std::mutex> lock(_pollingMutex);

    while (!_stopPolling) {
        auto elapsed = millis() - _lastStatusPoll;

        if (_lastStatusPoll > 0 && elapsed < STATUS_POLL_INTERVAL_MS) {
            auto sleepMs = STATUS_POLL_INTERVAL_MS - elapsed;
            _pollingCv.wait_for(lock, std::chrono::milliseconds(sleepMs),
                    [this] { return _stopPolling; });
            continue;
        }

        _lastStatusPoll = millis();
        lock.unlock();
        pollStatus();
        lock.lock();
    }
}

void Provider::pollStatus()
{
    if (!_statusGetter) { return; }

    auto result = _statusGetter->performGetRequest();

    if (!result) {
        DTU_LOGW("Failed to poll Shelly status: %s", _statusGetter->getErrorText());
        return;
    }

    auto pStream = result.getStream();
    if (!pStream) { return; }

    JsonDocument doc;
    if (deserializeJson(doc, *pStream) != DeserializationError::Ok) { return; }

    // Extract switch state and optional power metering
    bool isOn = false;
    if (doc["relays"].is<JsonArray>() && doc["relays"][0]["ison"].is<bool>()) {
        isOn = doc["relays"][0]["ison"].as<bool>();
    }

    std::optional<float> measuredPower;
    if (doc["meters"].is<JsonArray>() && doc["meters"][0]["power"].is<float>()) {
        measuredPower = doc["meters"][0]["power"].as<float>();
    }

    auto const& config = Configuration.get();

    {
        std::lock_guard<std::mutex> lock(_stats->_mutex);
        _stats->_isOn = isOn;
        _stats->_measuredPowerW = measuredPower;
        _stats->_ratedPowerW = config.GridCharger.Shelly.FixedAmperage * AC_VOLTAGE;
        _stats->_lastUpdate = millis();
    }

    // Sync local state with what Shelly actually reports
    _isOn = isOn;
}

} // namespace GridChargers::Shelly
