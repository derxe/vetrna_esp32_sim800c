#ifndef MAG_DIR_SENSOR_3D_H
#define MAG_DIR_SENSOR_3D_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

// TLV493D-A1B6 default is 0xBC/0xBD write/read, i.e. 0x5E as a 7-bit Wire address.
#define MAG_DIR_SENSOR_3D_A1B6_DEFAULT_I2C_ADDRESS 0x5E //0x1F 

#ifndef MAG_DIR_SENSOR_3D_I2C_ADDRESS
#define MAG_DIR_SENSOR_3D_I2C_ADDRESS MAG_DIR_SENSOR_3D_A1B6_DEFAULT_I2C_ADDRESS
#endif

//#define PRINT_DEBUG

#ifdef PRINT_DEBUG
  #define DBG_MAG(...) do { __VA_ARGS__; } while (0)
#else
  #define DBG_MAG(...) do {} while (0)
#endif


class MagDirSensor3D {
public:
    enum ReadStatus {
        OK = 0,
        PING_FAILED         = -1, 
        ALL_VALUES_ZERO     = -2,
        MAGNET_TOO_WEAK     = -3,
        WIRE_READ_FAILED    = -4,
        ONLY_VALUES_FF      = -5,
        ONLY_VALUES_00      = -6,
        WRITE_CONFIG_FAILED = -7,
        READ_CONFIG_FAILED  = -8,
        SDA_OR_SCL_NOT_CONN = -9
    };

    MagDirSensor3D(uint8_t sdaPin, uint8_t sclPin, uint8_t powerPin)
        : sdaPin_(sdaPin),
          sclPin_(sclPin),
          powerPin_(powerPin) {
    }

    void begin(uint32_t frequency = 100000UL) {
        frequency_ = frequency;
        pinMode(powerPin_, OUTPUT);
        loadNorthOffset();
    }

    void saveNorthOffset() {
        Preferences prefs;
        if (!prefs.begin(kPrefsNamespace, false)) {
            return;
        }
        prefs.putUShort(kNorthOffsetKey, northOffsetDeg_);
        prefs.end();
    }

    void loadNorthOffset() {
        Preferences prefs;
        if (!prefs.begin(kPrefsNamespace, true)) {
            return;
        }
        northOffsetDeg_ = prefs.getUShort(kNorthOffsetKey, 0);
        prefs.end();
    }

    void setNorthOffset(uint16_t northOffsetDeg) {
        northOffsetDeg_ = northOffsetDeg % 360;
    }

    uint16_t getNorthOffset() const {
        return northOffsetDeg_;
    }

    void enablePower(bool enabled) {
        if(enabled) {
            digitalWrite(powerPin_, LOW);
            pinMode(sdaPin_, INPUT);
            pinMode(sclPin_, INPUT);
        } else {
            Wire1.end();
            configured_ = false;
            digitalWrite(powerPin_, HIGH);
            pinMode(sclPin_, INPUT_PULLDOWN);
            pinMode(sdaPin_, INPUT_PULLDOWN);
        }
    }

    bool isConnected() {
        ReadStatus readStatus = enable_power_and_check_SDA_SCL_connected();
        enablePower(false);
        return readStatus == ReadStatus::OK;
    }

    ReadStatus read() {
        ReadStatus readStatus;
        
        readStatus = enable_power_and_check_SDA_SCL_connected();
        if (readStatus != ReadStatus::OK) {
            enablePower(false);
            return readStatus;
        }

        if(!ping()) {
            enablePower(false);
            return ReadStatus::PING_FAILED; 
        }

        readStatus = getMagneticFieldAndTemperature(&x_, &y_, &z_, &t_);
        enablePower(false);
        
        if (readStatus != ReadStatus::OK) return readStatus;

        DBG_MAG( Serial1.printf("Got read: x:%f y:%f z:%f t:%f\r\n", x_, y_, z_, t_) );
        if(t_ == 0 && x_==0 && y_==0 && z_==0) {
            // if the measurements are all 0 we can suspect that it didnt read anyting
            return ReadStatus::ALL_VALUES_ZERO; 
        }

        const float magPower = getMagnetPower();
        if(magPower < 2) return ReadStatus::MAGNET_TOO_WEAK;

        return ReadStatus::OK;
    }

    int getDirection() const {
        constexpr float twoPi = 6.28318530718f;

        float angle = atan2f(static_cast<float>(z_), static_cast<float>(x_));
        angle = -angle;   // flip to clockwise rotation, so that N:0, E:90 S:180, W:270
        angle /= twoPi;

        if (angle < 0.0f) {
            angle += 1.0f;
        }

        return ((int)(angle * 360) + 360 - northOffsetDeg_) % 360;
    }

    float getMagnetPower() const {
        const float x = static_cast<float>(x_);
        const float z = static_cast<float>(z_);
        return sqrtf((x * x) + (z * z));
    }

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getZ() const { return z_; }
    double getTemperature() const { return t_; }

private:
    static constexpr const char *kPrefsNamespace = "mag3d";
    static constexpr const char *kNorthOffsetKey = "north";

    static constexpr uint8_t kI2cAddress = MAG_DIR_SENSOR_3D_I2C_ADDRESS;
    static constexpr uint8_t kReadRegisterCount = 10;

    uint16_t northOffsetDeg_ = 0;
    uint8_t sdaPin_;
    uint8_t sclPin_;
    uint8_t powerPin_;
    uint32_t frequency_ = 400000UL;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double t_ = 0.0;
    bool configured_ = false;

    ReadStatus enable_power_and_check_SDA_SCL_connected() {
        Wire1.end();

        enablePower(true);
        pinMode(sclPin_, INPUT_PULLDOWN);
        pinMode(sdaPin_, INPUT_PULLDOWN);
        delayMicroseconds(20);

        const int sdaRead = digitalRead(sdaPin_);
        const int sclRead = digitalRead(sclPin_);

        pinMode(sdaPin_, INPUT);
        pinMode(sclPin_, INPUT);
        Wire1.begin(sdaPin_, sclPin_, frequency_);
        delayMicroseconds(200);

        if(!sdaRead || !sclRead) return ReadStatus::SDA_OR_SCL_NOT_CONN;

        return ReadStatus::OK;
    }

    bool ping() {
        Wire1.beginTransmission(kI2cAddress);
        int8_t status = Wire1.endTransmission();
        DBG_MAG( Serial1.printf("Pinging mag sensor status: %d\r\n", status) );
        return status == 0;
    }

    bool writeConfigRegisters(const uint8_t *config) {
        Wire1.beginTransmission(kI2cAddress);
        for (uint8_t i = 0; i < 4; ++i) {
            Wire1.write(config[i]);
        }

        const int8_t status = Wire1.endTransmission();
        DBG_MAG( Serial1.printf("3d mag sensor write config: %02X %02X %02X %02X status:%d\r\n",
                       config[0], config[1], config[2], config[3], status) );

        /*
        0: success.
        1: data too long to fit in transmit buffer.
        2: received NACK on transmit of address.
        3: received NACK on transmit of data.
        4: other error.
        5: timeout. 
        */

        return status == 0;
    }

    ReadStatus configureMasterControlledMode() {
        Serial.println("Configuring master controll mode");

        const bool readRegstersBeforeConfig = false; 
        uint8_t config[4] = {0x00, 0x03, 0x04, 0x60};

        if(readRegstersBeforeConfig)  {
            uint8_t registers[kReadRegisterCount] = {};

            if (readRegisters(registers) != ReadStatus::OK) {
                Serial.println("Unable to read config registers!");
                return ReadStatus::READ_CONFIG_FAILED;
            }

            uint8_t config[4] = {0x00, 0x03, 0x04, 0x60};
            
            
            config[0] = 0x00;

            // bit 7     P       parity
            // bits 6:5  IICADR  address select
            // bits 4:3  reserved/factory bits
            // bit 2     INT
            // bit 1     FAST
            // bit 0     LOW_POWER
            config[1] = static_cast<uint8_t>((registers[7] & 0x18) | 0x03);
            
            config[2] = registers[8];
            config[3] = static_cast<uint8_t>((registers[9] & 0x1F) | 0x60);

            config[1] &= 0x7F;
            if (!calculateParity(config[0] ^ config[1] ^ config[2] ^ config[3])) {
                config[1] |= 0x80;
            }
        }

        configured_ = writeConfigRegisters(config);
        delay(1);

        if(!configured_) {
            return ReadStatus::WRITE_CONFIG_FAILED;
        }
        return OK;
    }

    ReadStatus readRegisters(uint8_t *registers) {
        const uint8_t bytesRead = Wire1.requestFrom(kI2cAddress, kReadRegisterCount);
        //Serial1.printf("3d mag sensor reading regs count:%d bytesRead:%d\r\n", kReadRegisterCount, bytesRead);
        if (bytesRead != kReadRegisterCount) {
            while (Wire1.available()) Wire1.read();

            return ReadStatus::WIRE_READ_FAILED;
        }
        
        DBG_MAG( Serial1.println("Register read:") );
        for (uint8_t i = 0; i < kReadRegisterCount; ++i) {
            registers[i] = static_cast<uint8_t>(Wire1.read());
            DBG_MAG( Serial1.printf("* %d: %d\r\n", i, registers[i]) );
        }

        if(hasOnlyValue(registers, 0xFF)) {
            return ReadStatus::ONLY_VALUES_FF;
        }

        if(hasOnlyValue(registers, 0x00)) {
            return ReadStatus::ONLY_VALUES_00;
        }
        
        return ReadStatus::OK;
    }


    ReadStatus getMagneticFieldAndTemperature(double *x, double *y, double *z, double *temperature) {
        constexpr double magneticFieldMtPerLsb = 0.098;
        ReadStatus readStatus;

        uint8_t registers[kReadRegisterCount] = {};

        if (!configured_) {
            readStatus = configureMasterControlledMode();
            if(readStatus != ReadStatus::OK) return readStatus;
        }
        

        readStatus = readRegisters(registers);
        if (readStatus != ReadStatus::OK) {
            return readStatus;
        }

        /*
        Registers being read from the sensor:
        reg0: Bx[11:4]
        reg1: By[11:4]
        reg2: Bz[11:4]
        reg3:
            bits 7..4 = Temp[11:8]
            bits 3..2 = FRM
            bits 1..0 = CH
        reg4:
            bits 7..4 = Bx[3:0]
            bits 3..0 = By[3:0]
        reg5:
            bit 6     = T status bit
            bit 5     = FF fuse/parity status
            bit 4     = PD power-down/status bit
            bits 3..0 = Bz[3:0]
        reg6: Temp[7:0]
        reg7..9: reserved/fuse values used by the library when writing config
        */


        const int16_t rawX = signExtend12((static_cast<uint16_t>(registers[0]) << 4) | ((registers[4] & 0xF0) >> 4));
        const int16_t rawY = signExtend12((static_cast<uint16_t>(registers[1]) << 4) | (registers[4] & 0x0F));
        const int16_t rawZ = signExtend12((static_cast<uint16_t>(registers[2]) << 4) | (registers[5] & 0x0F));

        *x = static_cast<double>(rawX) * magneticFieldMtPerLsb;
        *y = static_cast<double>(rawY) * magneticFieldMtPerLsb;
        *z = static_cast<double>(rawZ) * magneticFieldMtPerLsb;

        const int16_t rawTempReading = signExtend12(((static_cast<uint16_t>(registers[3]) & 0xF0) << 4) | registers[6]);
        *temperature = calcTemperature(rawTempReading); 
        
        const uint8_t frm = (registers[3] & 0x0C) >> 2;

        // Must be “00” at readout to ensure X/Y/Z/T come from the same conversion. Else, conversion is running.
        // If “00” no conversion (internal power-down) or xdirection conversion started (but value not yet stored in the register)
        // If “01” y-direction conversion ongoing
        // If “10” z-direction conversion ongoing
        // If “11” temperature conversion 
        const uint8_t ch = registers[3] & 0x03;

        // Must be “0” at readout. Provides a flag to signal that the sensor is not in normal operating mode
        // T = “1” → data is tampered e.g. due to an inadvertent test mode, try sensor reset
        // T = “0” → data is valid (or not updated)
        const uint8_t t = (registers[5] & 0x40) >> 6;

        // Provides a flag from the internal fuse parity check. This flag is only valid if the PT bit (parity test enabled) in register Mod2 is enabled as well.
        // FF = “1” → fuse setup OK
        // FF = “0” → fuse setup not OK, try sensor reset
        const uint8_t ff = (registers[5] & 0x20) >> 5;

        // Must be “1” at readout.
        // If “1”, Bx, By, Bz and Temp conversion completed.
        // If “0” Bx, By, Bz and Temp conversion running.
        const uint8_t pd = (registers[5] & 0x10) >> 4;

        DBG_MAG( Serial1.printf("FRM:%u CH:%u T:%u FF:%u PD:%u\r\n", frm, ch, t, ff, pd) );
        return ReadStatus::OK;
    }

    static double calcTemperature(int16_t rawTempReading) {
        constexpr double tempCPerLsb = 1.10;
        constexpr double tempOffsetLsb = 340.0;
        constexpr double tempReferenceC = 25.0;

        return ((static_cast<double>(rawTempReading) - tempOffsetLsb) * tempCPerLsb) + tempReferenceC;
    }

    static bool hasOnlyValue(const uint8_t *registers, uint8_t value) {
        for (uint8_t i = 0; i < kReadRegisterCount; ++i) {
            if (registers[i] != value) {
                return false;
            }
        }
        return true;
    }

    static uint8_t calculateParity(uint8_t value) {
        value ^= value >> 4;
        value ^= value >> 2;
        value ^= value >> 1;
        return value & 1U;
    }

    static int16_t signExtend12(uint16_t value) {
        value &= 0x0FFF;
        if (value & 0x0800) {
            value |= 0xF000;
        }
        return static_cast<int16_t>(value);
    }
};

#endif

