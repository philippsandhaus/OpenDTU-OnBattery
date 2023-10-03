/* frameHandler.h
 *
 * Arduino library to read from Victron devices using VE.Direct protocol.
 * Derived from Victron framehandler reference implementation.
 *
 * 2020.05.05 - 0.2 - initial release
 * 2021.02.23 - 0.3 - change frameLen to 22 per VE.Direct Protocol version 3.30
 * 2022.08.20 - 0.4 - changes for OpenDTU
 *
 */

#pragma once

#include <Arduino.h>
#include <array>
#include <map>
#include <memory>

#define VE_MAX_VALUE_LEN 33 // VE.Direct Protocol: max value size is 33 including /0
#define VE_MAX_HEX_LEN 100 // Maximum size of hex frame - max payload 34 byte (=68 char) + safe buffer


// hex commands
enum VeDirectHexCommand {
    ENTER_BOOT = 0x00,
    PING = 0x01,
    APP_VERSION = 0x02,
    PRODUCT_ID = 0x04,
    RESTART = 0x06,
    GET = 0x07,
    SET = 0x08,
    ASYNC = 0x0A
};

// hex ids
enum VeDirectHexId {
    BATTERY_MAXIMUM_CURRENT = 0xEDF0,
    CHARGER_MAXIMUM_CURRENT = 0xEDDF,
    CHARGER_CURRENT = 0xEDD7,
    CHARGER_VOLTAGE = 0xEDD5,
    CHARGER_ERROR_CODE = 0xEDDA,
};

enum VeDirectFlag {
    DEFAULT_FLAG0 = 0x00,
};

class VeDirectFrameHandler {
public:
    VeDirectFrameHandler();
    virtual void init(int8_t rx, int8_t tx, Print* msgOut, bool verboseLogging, uint16_t hwSerialPort);
    void loop();                                 // main loop to read ve.direct data
    uint32_t getLastUpdate() const;              // timestamp of last successful frame read

protected:
    bool _verboseLogging;
    Print* _msgOut;
    uint32_t _lastUpdate;
    typedef struct {
        uint16_t PID = 0;               // product id
        char SER[VE_MAX_VALUE_LEN];     // serial number
        char FW[VE_MAX_VALUE_LEN];      // firmware release number
        double V = 0;                   // battery voltage in V
        double I = 0;                   // battery current in A
        double E = 0;                   // efficiency in percent (calculated, moving average)

        String getPidAsString() const;  // product id as string
    } veStruct;

    void sendHexCommand(VeDirectHexCommand cmd, VeDirectHexId id, VeDirectFlag flag, uint16_t value);
    bool textRxEvent(std::string const& who, char* name, char* value, veStruct& frame);
    bool isDataValid(veStruct const& frame) const;      // return true if data valid and not outdated

    template<typename T>
    static String const& getAsString(std::map<T, String> const& values, T val);

private:
    void setLastUpdate();                     // set timestampt after successful frame read
    void dumpDebugBuffer();
    void rxData(uint8_t inbyte);              // byte of serial data
    virtual void textRxEvent(char *, char *) = 0;
    virtual void frameValidEvent() = 0;
    int hexRxEvent(uint8_t);
    
    std::unique_ptr<HardwareSerial> _vedirectSerial;
    int _state;                                // current state
    int _prevState;                            // previous state
    uint8_t _checksum;                         // checksum value
    char * _textPointer;                       // pointer to the private buffer we're writing to, name or value
    int _hexSize;                              // length of hex buffer
    char _hexBuffer[VE_MAX_HEX_LEN] = { };	   // buffer for received hex frames
    char _name[VE_MAX_VALUE_LEN];              // buffer for the field name
    char _value[VE_MAX_VALUE_LEN];             // buffer for the field value
    std::array<uint8_t, 512> _debugBuffer;
    unsigned _debugIn;
    uint32_t _lastByteMillis;
};
