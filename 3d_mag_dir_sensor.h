#ifndef MAG_DIR_SENSOR_3D_H
#define MAG_DIR_SENSOR_3D_H

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

#include "TLx493D_inc.hpp"

class MagDirSensor3D {
public:
    MagDirSensor3D(uint8_t sdaPin, uint8_t sclPin, uint8_t powerPin)
        : sdaPin_(sdaPin),
          sclPin_(sclPin),
          powerPin_(powerPin),
          dut_(Wire1, TLx493D_IIC_ADDR_A0_e) {
    }

    bool begin(uint32_t frequency = 400000UL) {
        frequency_ = frequency;
        pinMode(powerPin_, OUTPUT);
        setPower(true);
        Wire1.begin(sdaPin_, sclPin_, frequency_);
        loadNorthOffset();
        return dut_.begin();
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

    void setPower(bool enabled) {
        digitalWrite(powerPin_, enabled ? LOW : HIGH);
    }

    bool isConnected() {
        Wire1.end();

        setPower(true);
        pinMode(sclPin_, INPUT_PULLDOWN);
        pinMode(sdaPin_, INPUT_PULLDOWN);
        delayMicroseconds(20);

        const int sdaRead = digitalRead(sdaPin_);
        const int sclRead = digitalRead(sclPin_);

        pinMode(sdaPin_, INPUT);
        pinMode(sclPin_, INPUT);
        Wire1.begin(sdaPin_, sclPin_, frequency_);
        dut_.begin();

        return sdaRead && sclRead;
    }

    bool read() {
        dut_.getMagneticFieldAndTemperature(&x_, &y_, &z_, &t_);
        if(t_ == 0 && x_==0 && y_==0 && z_==0) return false; // if the measurements are all 0 we can suspect that it didnt read anyting
        return true; // return dut_.hasValidData();
    }

    int getDirection() const {
        constexpr float twoPi = 6.28318530718f;

        float angle = atan2f(static_cast<float>(z_), static_cast<float>(x_));
        angle = -angle;   // flip to clockwise rotation, so N:0, E:90 S:180, W:270
        angle /= twoPi;

        if (angle < 0.0f) {
            angle += 1.0f;
        }

        return ((int)(angle * 360) + 360 - northOffsetDeg_) % 360;
    }

    float getPower() const {
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

    uint16_t northOffsetDeg_ = 0;
    uint8_t sdaPin_;
    uint8_t sclPin_;
    uint8_t powerPin_;
    uint32_t frequency_ = 400000UL;

    ifx::tlx493d::TLx493D_A1B6 dut_;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double t_ = 0.0;
};

#endif
