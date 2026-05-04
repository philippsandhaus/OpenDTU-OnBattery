// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <gridcharger/Provider.h>
#include <gridcharger/shelly/Stats.h>
#include <HttpGetter.h>

namespace GridChargers::Shelly {

class Provider : public ::GridChargers::Provider {
public:
    bool init() final;
    void deinit() final;
    void loop() final;
    std::shared_ptr<::GridChargers::Stats> getStats() const final { return _stats; }

    bool getAutoPowerStatus() const final { return _isOn; }

private:
    void powerControlLoop();

    static void pollingLoopHelper(void* context);
    void pollingLoop();
    void pollStatus();
    void sendSwitchCommand(bool on);

    static constexpr int STATUS_POLL_INTERVAL_MS = 5000;
    static constexpr int CONTROL_INTERVAL_MS = 10000;
    static constexpr int HTTP_TIMEOUT_MS = 2000;
    // Minimum time between switch state changes to avoid relay wear
    static constexpr int MIN_SWITCH_INTERVAL_MS = 30000;
    // AC mains voltage assumed for rated power calculation (V)
    static constexpr float AC_VOLTAGE = 230.0f;

    TaskHandle_t _pollingTaskHandle = nullptr;
    std::atomic<bool> _pollingTaskDone = false;
    bool _stopPolling = false;
    mutable std::mutex _pollingMutex;
    std::condition_variable _pollingCv;
    uint32_t _lastStatusPoll = 0;

    std::unique_ptr<HttpRequestConfig> _statusRequestConfig;
    std::unique_ptr<HttpGetter> _statusGetter;

    String _controlBaseUrl;  // e.g. http://192.168.1.100

    bool _isOn = false;
    bool _targetState = false;
    uint32_t _lastSwitchMillis = 0;
    uint32_t _lastControlMillis = 0;

    bool _batteryEmergencyCharging = false;

    std::shared_ptr<Stats> _stats = std::make_shared<Stats>();
};

} // namespace GridChargers::Shelly
