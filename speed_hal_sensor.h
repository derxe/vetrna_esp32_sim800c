class SpeedHalSensor {
private:
    const int sensorPin;
    const int powerPin;

    bool hasPower = false;

public:
    SpeedHalSensor(int sensorPin, int powerPin)
        : sensorPin(sensorPin), powerPin(powerPin) {}

    void begin() {
        pinMode(powerPin, OUTPUT);
        setPower(true);
    }

    bool isConnected() {
        pinMode(sensorPin, INPUT_PULLDOWN);
        int readDown = digitalRead(sensorPin);

        pinMode(sensorPin, INPUT_PULLUP);
        int readUp = digitalRead(sensorPin);

        // restore senorPin Pull up/down configuration
        setPower(hasPower);

        if(readDown == 0 && readUp == 0) return true;  // magnet present
        if(readDown == 1 && readUp == 1) return true;  // magnet not present

        return false; // since there is no chip changing the sensorPin, the pull down should be 0 and pull up should 1
    }

    void setPower(bool enable) {
        hasPower = enable;

        if(enable) {
            digitalWrite(powerPin, LOW);
            pinMode(sensorPin, INPUT_PULLUP);
        } else {
            digitalWrite(powerPin, HIGH);
            pinMode(sensorPin, INPUT_PULLDOWN);
        }
    }

    int read() {
        if(!hasPower) return -1;
        return digitalRead(sensorPin);
    }
};