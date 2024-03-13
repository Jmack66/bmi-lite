#pragma once

#include "bmi_definitions.h"
#include "Wire.h"
#include "vector.h"
// Thanks to Seed studio for their library which this is influenced by
// https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Gyroscope_BMI088/blob/master/BMI088.h




struct Imu{

    float accRange = 0.0f;
    float gyroRange = 0.0f;
    TwoWire* internal_bus = nullptr;
    bool use_alt = false;
    uint8_t acc_addr = 0;
    uint8_t gyro_addr = 0;

    Imu(TwoWire* bus, bool use_alt = false) : internal_bus(bus), use_alt(use_alt) {
        if(use_alt){
            acc_addr = BMI088_ACC_ALT_ADDRESS;
            gyro_addr = BMI088_GYRO_ALT_ADDRESS;
        }else{
            acc_addr = BMI088_ACC_ADDRESS;
            gyro_addr = BMI088_GYRO_ADDRESS;
        }
    }


    void begin(){
        setAccScaleRange(RANGE_6G);
        setAccOutputDataRate(ODR_100);
        setAccPoweMode(ACC_ACTIVE);
        setGyroScaleRange(RANGE_250);
        setGyroOutputDataRate(ODR_2000_BW_532);
        setGyroPoweMode(GYRO_NORMAL); 
    }

    void setAccScaleRange(acc_scale_type_t range) {
        if (range == RANGE_3G) {
            accRange = 3.0f * BMI088_G_CONST;
        } else if (range == RANGE_6G) {
            accRange = 6.0f * BMI088_G_CONST;
        } else if (range == RANGE_12G) {
            accRange = 12.0f * BMI088_G_CONST;
        } else if (range == RANGE_24G) {
            accRange = 24.0f * BMI088_G_CONST;
        }
        auto check = writeByte(acc_addr, BMI088_ACC_RANGE, (uint8_t)range);
        //assert(check != 0,"INIT OF ONBOARD IMU FAILED");
    }

    void resetAcc(void) {
        writeByte(acc_addr, BMI088_ACC_SOFT_RESET, 0xB6);
    }

    void resetGyro(void) {
        writeByte(gyro_addr, BMI088_GYRO_SOFT_RESET, 0xB6);
    }

    void setAccPoweMode(acc_power_type_t mode) {
        if (mode == ACC_ACTIVE) {
            writeByte(acc_addr, BMI088_ACC_PWR_CTRl, 0x04);
            writeByte(acc_addr, BMI088_ACC_PWR_CONF, 0x00);
        } else if (mode == ACC_SUSPEND) {
            writeByte(acc_addr, BMI088_ACC_PWR_CONF, 0x03);
            writeByte(acc_addr, BMI088_ACC_PWR_CTRl, 0x00);
        }
    }

    void setGyroPoweMode(gyro_power_type_t mode) {
        if (mode == GYRO_NORMAL) {
            writeByte( gyro_addr, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
        } else if (mode == GYRO_SUSPEND) {
            writeByte( gyro_addr, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
        } else if (mode == GYRO_DEEP_SUSPEND) {
            writeByte( gyro_addr, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
        }
    }


    void setAccOutputDataRate(acc_odr_type_t odr) {
        uint8_t data = 0;

        data = readByte(acc_addr, BMI088_ACC_CONF);
        data = data & 0xf0;
        data = data | (uint8_t)odr;

        writeByte(acc_addr, BMI088_ACC_CONF, data);
    }

    void setGyroScaleRange(gyro_scale_type_t range) {
        if (range == RANGE_2000) {
            gyroRange = 2000;
        } else if (range == RANGE_1000) {
            gyroRange = 1000;
        } else if (range == RANGE_500) {
            gyroRange = 500;
        } else if (range == RANGE_250) {
            gyroRange = 250;
        } else if (range == RANGE_125) {
            gyroRange = 125;
        }

        writeByte(gyro_addr, BMI088_GYRO_RANGE, (uint8_t)range);

    }

    void setGyroOutputDataRate(gyro_odr_type_t odr) {
        writeByte(gyro_addr, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
        }

    Vec getGyro(){
        Vec out;
        uint8_t buf[6] = {0};
        uint16_t gx = 0, gy = 0, gz = 0;
        float value = 0;

        read(gyro_addr, BMI088_GYRO_RATE_X_LSB, buf, 6);

        gx = buf[0] | (buf[1] << 8);
        gy = buf[2] | (buf[3] << 8);
        gz = buf[4] | (buf[5] << 8);

        value = (int16_t)gx;
        out.x = gyroRange * value / 32767;

        value = (int16_t)gy;
        out.y = gyroRange * value / 32767;

        value = (int16_t)gz;
        out.z = gyroRange * value / 32767;
        return out;
    }

    Vec getAccel(){
        Vec out;
        uint8_t buf[6] = {0};
        uint16_t ax = 0, ay = 0, az = 0;
        float value = 0;

        read(acc_addr, BMI088_ACC_X_LSB, buf, 6);

        ax = buf[0] | (buf[1] << 8);
        ay = buf[2] | (buf[3] << 8);
        az = buf[4] | (buf[5] << 8);

        value = (int16_t)ax;
        out.x = accRange * value / 32767;

        value = (int16_t)ay;
        out.y = accRange * value / 32767;

        value = (int16_t)az;
        out.z = accRange * value / 32767;
        return out;
    }


    void read(uint8_t addr, uint8_t reg, uint8_t* buf, uint16_t len) {
        internal_bus->beginTransmission(addr);
        internal_bus->write(reg);
        internal_bus->endTransmission();

        internal_bus->requestFrom(addr, len);
        while (internal_bus->available()) {
            for (uint16_t i = 0; i < len; i ++) {
                buf[i] = internal_bus->read();
            }
        }
    }

    int writeByte(uint8_t addr, uint8_t reg, uint8_t data){
        internal_bus->beginTransmission(addr);
        internal_bus->write(reg);
        internal_bus->write(data);
        auto sad = internal_bus->endTransmission();
        return sad;
    }

    uint8_t readByte(uint8_t addr, uint8_t reg){
        internal_bus->beginTransmission(addr);
        internal_bus->write(reg);
        internal_bus->endTransmission();

        internal_bus->requestFrom((uint8_t)addr, (uint8_t)1);
        return internal_bus->read();
    }


};
