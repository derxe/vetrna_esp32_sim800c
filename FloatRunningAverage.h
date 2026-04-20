#pragma once


template <int BUF_LEN>
class FloatRunningAverage {
public:
    FloatRunningAverage()
        : head(0), count(0), sum(0.0f) {}

    void addSample(float val) {
        //Serial_print("favg read:");
        //Serial_println(String(val, 3));

        if (count < BUF_LEN) {
            buffer[head] = val;
            sum += val;
            head = (head + 1) % BUF_LEN;
            count++;
        } else {
            float old = buffer[head];
            sum -= old;
            buffer[head] = val;
            sum += val;
            head = (head + 1) % BUF_LEN;
        }
    }

    // return average value 
    float get() {
        if (count == 0) return NAN;
        return sum / (float)count;
    }
    
    int size() {
    	return count;
    }

    bool isEmpty() const { return count == 0; }

    void reset() {
        head = 0;
        count = 0;
        sum = 0.0f;
    }

private:
    float buffer[BUF_LEN];
    int head;
    int count;
    float sum;
};























