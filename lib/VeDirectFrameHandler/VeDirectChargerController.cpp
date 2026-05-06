// SPDX-License-Identifier: GPL-2.0-or-later

#include <Arduino.h>
#include "VeDirectChargerController.h"
#include <LogHelper.h>

#undef TAG
static const char* TAG = "veDirect";
#define SUBTAG _logId

void VeDirectChargerController::init(gpio_num_t rx, gpio_num_t tx, uint8_t hwSerialPort)
{
    VeDirectFrameHandler::init("Charger", rx, tx, hwSerialPort);
}

bool VeDirectChargerController::processTextDataDerived(std::string const& name, std::string const& value)
{
    if (name == "CS") {
        _tmpFrame.currentState_CS = static_cast<uint8_t>(atoi(value.c_str()));
        return true;
    }
    if (name == "ERR") {
        _tmpFrame.errorCode_ERR = static_cast<uint8_t>(atoi(value.c_str()));
        return true;
    }
    if (name == "OR") {
        _tmpFrame.offReason_OR = static_cast<uint32_t>(strtoul(value.c_str(), nullptr, 0));
        return true;
    }
    if (name == "MODE") {
        _tmpFrame.deviceMode_MODE = static_cast<uint8_t>(atoi(value.c_str()));
        return true;
    }
    return false;
}

void VeDirectChargerController::frameValidEvent()
{
    _tmpFrame.outputPower_W = static_cast<int16_t>(
        (_tmpFrame.batteryVoltage_V_mV / 1000.0f) * (_tmpFrame.batteryCurrent_I_mA / 1000.0f));
}

void VeDirectChargerController::loop()
{
    if (isHexCommandPossible()) {
        sendNextHexCommandFromQueue();
    }

    VeDirectFrameHandler::loop();

    if (!isHexCommandPossible()) { return; }

    auto resetTimestamp = [this](auto& pair) {
        if (pair.first > 0 && (millis() - pair.first) > (10 * 1000)) {
            pair.first = 0;
        }
    };

    resetTimestamp(_tmpFrame.ChargerVoltageMilliVolt);
    resetTimestamp(_tmpFrame.ChargerCurrentMilliAmp);
}

bool VeDirectChargerController::hexDataHandler(VeDirectHexData const& data)
{
    if (data.rsp != VeDirectHexResponse::GET &&
        data.rsp != VeDirectHexResponse::SET &&
        data.rsp != VeDirectHexResponse::ASYNC) { return false; }

    if ((data.rsp == VeDirectHexResponse::GET || data.rsp == VeDirectHexResponse::SET) &&
        (data.addr == _hexQueue[_sendQueueNr]._hexRegister)) {
        _sendTimeout = 0;
    }

    if (data.rsp == VeDirectHexResponse::SET) {
        switch (data.addr) {
            case VeDirectHexRegister::ChargeCurrentLimit: return true;
            case VeDirectHexRegister::DeviceMode:         return true;
            default: return false;
        }
    }

    auto regLog = static_cast<uint16_t>(data.addr);

    switch (data.addr) {
        case VeDirectHexRegister::ChargerVoltage:
            // Unit: 10 mV/bit
            _tmpFrame.ChargerVoltageMilliVolt = { millis(), data.value * 10 };
            DTU_LOGD("Hex Data: ChargerVoltage (0x%04X): %.2f V",
                    regLog, _tmpFrame.ChargerVoltageMilliVolt.second / 1000.0);
            return true;

        case VeDirectHexRegister::ChargerCurrent:
            // Unit: 100 mA/bit
            _tmpFrame.ChargerCurrentMilliAmp = { millis(), data.value * 100 };
            DTU_LOGD("Hex Data: ChargerCurrent (0x%04X): %.2f A",
                    regLog, _tmpFrame.ChargerCurrentMilliAmp.second / 1000.0);
            return true;

        case VeDirectHexRegister::DeviceState:
            _tmpFrame.currentState_CS = static_cast<uint8_t>(data.value);
            DTU_LOGD("Hex Data: DeviceState (0x%04X): %u", regLog, data.value);
            return true;

        default:
            return false;
    }
}

bool VeDirectChargerController::isHexCommandPossible() const
{
    return _canSend && (_tmpFrame.getFwVersionAsInteger() >= 153);
}

void VeDirectChargerController::sendNextHexCommandFromQueue()
{
    auto millisTime = millis();
    if (!isStateIdle() || (millisTime - _hexQueue[_sendQueueNr]._lastSendTime) <= _sendTimeout) {
        return;
    }

    for (int pass = 0; pass < 2; ++pass) {
        bool highPrio = (pass == 0);
        auto idx = _sendQueueNr + 1;
        if (idx >= _hexQueue.size()) { idx = 0; }

        do {
            auto& entry = _hexQueue[idx];
            bool isHighPrio = (entry._readPeriod == CHARGER_HIGH_PRIO_CMD);
            if (highPrio != isHighPrio) {
                ++idx;
                if (idx >= _hexQueue.size()) { idx = 0; }
                continue;
            }

            bool due = (millisTime - entry._lastSendTime) > (static_cast<uint32_t>(entry._readPeriod) * 1000);
            bool hasPendingSet = entry._setCommand && entry._data.has_value();

            if (due || hasPendingSet) {
                bool sent = false;
                if (entry._setCommand) {
                    if (entry._data.has_value()) {
                        sendHexCommand(VeDirectHexCommand::SET, entry._hexRegister,
                                       entry._data.value(), entry._dataLength);
                        entry._data.reset();
                        sent = true;
                    }
                } else {
                    sendHexCommand(VeDirectHexCommand::GET, entry._hexRegister);
                    sent = true;
                }

                if (sent) {
                    entry._lastSendTime = millisTime;
                    _sendTimeout = 500;
                    _sendQueueNr = idx;
                    return;
                }
            }

            ++idx;
            if (idx >= _hexQueue.size()) { idx = 0; }
        } while (idx != _sendQueueNr);
    }
}

void VeDirectChargerController::setChargeCurrent(float ampere)
{
    for (auto& entry : _hexQueue) {
        if (entry._hexRegister == VeDirectHexRegister::ChargeCurrentLimit) {
            // Convert A → register units (0.1 A/unit → multiply by 10)
            float scaled = ampere * 10.0f;
            if (scaled >= 0 && scaled <= UINT16_MAX) {
                entry._data = static_cast<uint32_t>(scaled);
                DTU_LOGD("Queued charge current setpoint: %.1f A (register value %u)",
                        ampere, entry._data.value());
            }
            return;
        }
    }
}

void VeDirectChargerController::setDeviceMode(bool on)
{
    // DeviceMode: 1 = Charger (on), 4 = Off
    uint32_t modeValue = on ? 1u : 4u;
    for (auto& entry : _hexQueue) {
        if (entry._hexRegister == VeDirectHexRegister::ChargeCurrentLimit) {
            // Re-use the SET slot pattern by queueing DeviceMode directly
            (void)modeValue; // sent below via sendHexCommand outside queue
            break;
        }
    }
    // Send immediately if idle; otherwise it will be picked up on the next loop tick.
    if (isHexCommandPossible() && isStateIdle()) {
        sendHexCommand(VeDirectHexCommand::SET, VeDirectHexRegister::DeviceMode, modeValue, 8);
        DTU_LOGI("Sent DeviceMode = %u (%s)", modeValue, on ? "on" : "off");
    }
}
