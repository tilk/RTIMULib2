////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULSM6DS33.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMULSM6DS33::RTIMULSM6DS33(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULSM6DS33::~RTIMULSM6DS33()
{
}

bool RTIMULSM6DS33::IMUInit()
{
    unsigned char result;

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    if (m_settings->HALRead(LSM6DS33_ADDRESS0, LSM6DS33_WHO_AM_I, 1, &result, "") && result == LSM6DS33_ID) {
        m_accelGyroSlaveAddr = LSM6DS33_ADDRESS0;
    } else if (m_settings->HALRead(LSM6DS33_ADDRESS1, LSM6DS33_WHO_AM_I, 1, &result, "") && result == LSM6DS33_ID) {
        m_accelGyroSlaveAddr = LSM6DS33_ADDRESS1;
    } else {
        HAL_ERROR("LSM6DS33 not detected");
        return false;
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro/accel

    if (!m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL1_XL, 0x80, "Failed to boot LSM6DS33 accel"))
        return false;

    if (!m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL2_G, 0x80, "Failed to boot LSM6DS33 gyro"))
        return false;

    m_settings->delayMs(100);

    if (!setAccelCTRL1())
            return false;

    if (!setGyroCTRL2())
            return false;
    
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    HAL_INFO("LSM6DS33 init complete\n");

    //  Set up the mag

    if (m_settings->HALRead(LIS3MDL_ADDRESS0, LIS3MDL_WHO_AM_I, 1, &result, "") && result == LIS3MDL_ID) {
        m_magSlaveAddr = LIS3MDL_ADDRESS0;
    } else if (m_settings->HALRead(LIS3MDL_ADDRESS1, LIS3MDL_WHO_AM_I, 1, &result, "") && result == LIS3MDL_ID) {
        m_magSlaveAddr = LIS3MDL_ADDRESS1;
    } else {
        HAL_ERROR("LIS3MDL not detected");
        return false;
    }

    if (!setCompassCTRL1())
        return false;

    if (!setCompassCTRL2())
        return false;

    if (!setCompassCTRL3())
        return false;

    if (!setCompassCTRL4())
        return false;
    
    gyroBiasInit();

    return true;
}

bool RTIMULSM6DS33::setAccelCTRL1()
{
    unsigned char ctrl1 = 
        (m_settings->m_LSM6DS33AccelSampleRate << 4) |
        (LSM6DS33_ACCEL_FSR_8G << 2) |
        (LSM6DS33_ACCEL_BANDWIDTH_400);

    int sr;

    switch (m_settings->m_LSM6DS33AccelSampleRate) {
    case LSM6DS33_ACCEL_SAMPLERATE_13: sr = 13; break;
    case LSM6DS33_ACCEL_SAMPLERATE_26: sr = 26; break;
    case LSM6DS33_ACCEL_SAMPLERATE_52: sr = 52; break;
    case LSM6DS33_ACCEL_SAMPLERATE_104: sr = 104; break;
    case LSM6DS33_ACCEL_SAMPLERATE_208: sr = 208; break;
    case LSM6DS33_ACCEL_SAMPLERATE_416: sr = 416; break;
    case LSM6DS33_ACCEL_SAMPLERATE_833: sr = 833; break;
    case LSM6DS33_ACCEL_SAMPLERATE_1660: sr = 1660; break;
    case LSM6DS33_ACCEL_SAMPLERATE_3330: sr = 3330; break;
    case LSM6DS33_ACCEL_SAMPLERATE_6660: sr = 6660; break;
    default:
        HAL_ERROR1("Illegal LSM6DS33 accel sample rate code %d\n", m_settings->m_LSM6DS33AccelSampleRate);
        return false;
    }

    if (sr > m_sampleRate) m_sampleRate = sr;
    
    switch (m_settings->m_LSM6DS33AccelBW) {
    case LSM6DS33_ACCEL_BANDWIDTH_400: break;
    case LSM6DS33_ACCEL_BANDWIDTH_200: break;
    case LSM6DS33_ACCEL_BANDWIDTH_100: break;
    case LSM6DS33_ACCEL_BANDWIDTH_50: break;
    default:
        HAL_ERROR1("Illegal LSM6DS33 accel bandwidth code %d\n", m_settings->m_LSM6DS33AccelBW);
        return false;
    }

    switch (m_settings->m_LSM6DS33AccelFsr) {
    case LSM6DS33_ACCEL_FSR_2G: m_accelScale = (RTFLOAT)2.0/0x8000; break;
    case LSM6DS33_ACCEL_FSR_16G: m_accelScale = (RTFLOAT)16.0/0x8000; break;
    case LSM6DS33_ACCEL_FSR_4G: m_accelScale = (RTFLOAT)4.0/0x8000; break;
    case LSM6DS33_ACCEL_FSR_8G: m_accelScale = (RTFLOAT)8.0/0x8000; break;
    default:
        HAL_ERROR1("Illegal LSM6DS33 accel FSR code %d\n", m_settings->m_LSM6DS33AccelFsr);
        return false;
    }

    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL1_XL, ctrl1, "Failed to set LSM6DS33 accel CTRL1"));
}

bool RTIMULSM6DS33::setGyroCTRL2()
{
    unsigned char ctrl2 =
        (m_settings->m_LSM6DS33GyroSampleRate << 4) |
        (m_settings->m_LSM6DS33GyroFsr << 2);

    int sr;

    switch (m_settings->m_LSM6DS33GyroSampleRate) {
    case LSM6DS33_GYRO_SAMPLERATE_13: sr = 13; break;
    case LSM6DS33_GYRO_SAMPLERATE_26: sr = 26; break;
    case LSM6DS33_GYRO_SAMPLERATE_52: sr = 52; break;
    case LSM6DS33_GYRO_SAMPLERATE_104: sr = 104; break;
    case LSM6DS33_GYRO_SAMPLERATE_208: sr = 208; break;
    case LSM6DS33_GYRO_SAMPLERATE_416: sr = 416; break;
    case LSM6DS33_GYRO_SAMPLERATE_833: sr = 833; break;
    case LSM6DS33_GYRO_SAMPLERATE_1660: sr = 1660; break;
    default:
        HAL_ERROR1("Illegal LSM6DS33 gyro sample rate code %d\n", m_settings->m_LSM6DS33GyroSampleRate);
        return false;
    }

    if (sr > m_sampleRate) m_sampleRate = sr;

    switch (m_settings->m_LSM6DS33GyroFsr) {
    case LSM6DS33_GYRO_FSR_245: m_gyroScale = (RTFLOAT)245.0/0x8000 * RTMATH_DEGREE_TO_RAD; break;
    case LSM6DS33_GYRO_FSR_500: m_gyroScale = (RTFLOAT)500.0/0x8000 * RTMATH_DEGREE_TO_RAD; break;
    case LSM6DS33_GYRO_FSR_1000: m_gyroScale = (RTFLOAT)1000.0/0x8000 * RTMATH_DEGREE_TO_RAD; break;
    case LSM6DS33_GYRO_FSR_2000: m_gyroScale = (RTFLOAT)2000.0/0x8000 * RTMATH_DEGREE_TO_RAD; break;
    default:
        HAL_ERROR1("Illegal LSM6DS33 gyro FSR code %d\n", m_settings->m_LSM6DS33GyroFsr);
        return false;
    }
    return (m_settings->HALWrite(m_accelGyroSlaveAddr, LSM6DS33_CTRL2_G, ctrl2, "Failed to set LSM6DS33 gyro CTRL2"));
}

bool RTIMULSM6DS33::setCompassCTRL1()
{
    unsigned char ctrl1 =
        (m_settings->m_LIS3MDLCompassXYMode << 5) |
        (m_settings->m_LIS3MDLCompassSampleRate << 2);

    int sr;

    switch (m_settings->m_LIS3MDLCompassSampleRate) {
    case LIS3MDL_COMPASS_SAMPLERATE_0_625: sr = 1; break;
    case LIS3MDL_COMPASS_SAMPLERATE_1_25: sr = 2; break;
    case LIS3MDL_COMPASS_SAMPLERATE_2_5: sr = 3; break;
    case LIS3MDL_COMPASS_SAMPLERATE_5: sr = 5; break;
    case LIS3MDL_COMPASS_SAMPLERATE_10: sr = 10; break;
    case LIS3MDL_COMPASS_SAMPLERATE_20: sr = 20; break;
    case LIS3MDL_COMPASS_SAMPLERATE_40: sr = 40; break;
    case LIS3MDL_COMPASS_SAMPLERATE_80: sr = 80; break;
    default:
        HAL_ERROR1("Illegal LIS3MDL compass sample rate code %d\n", m_settings->m_LIS3MDLCompassSampleRate);
        return false;
    }

    if (sr > m_sampleRate) m_sampleRate = sr;

    if (m_settings->m_LIS3MDLCompassXYMode < 0 || m_settings->m_LIS3MDLCompassXYMode > 0x3) {
        HAL_ERROR1("Illegal LIS3MDL XY operating mode code %d\n", m_settings->m_LIS3MDLCompassXYMode);
        return false;
    }

    return (m_settings->HALWrite(m_magSlaveAddr, LIS3MDL_CTRL_REG1, ctrl1, "Failed to set LIS3MDL compass CTRL1"));
}

bool RTIMULSM6DS33::setCompassCTRL2()
{
    unsigned char ctrl2 =
        (m_settings->m_LIS3MDLCompassFsr << 5);
    
    switch (m_settings->m_LIS3MDLCompassFsr) {
    case LIS3MDL_COMPASS_FSR_4: m_compassScale = (RTFLOAT)100*4.0/0x8000; break;
    case LIS3MDL_COMPASS_FSR_8: m_compassScale = (RTFLOAT)100*8.0/0x8000; break;
    case LIS3MDL_COMPASS_FSR_12: m_compassScale = (RTFLOAT)100*12.0/0x8000; break;
    case LIS3MDL_COMPASS_FSR_16: m_compassScale = (RTFLOAT)100*16.0/0x8000; break;
    default:
        HAL_ERROR1("Illegal LIS3MDL compass FSR code %d\n", m_settings->m_LIS3MDLCompassFsr);
        return false;
    }
    
    return (m_settings->HALWrite(m_magSlaveAddr, LIS3MDL_CTRL_REG2, ctrl2, "Failed to set LIS3MDL compass CTRL2"));
}

bool RTIMULSM6DS33::setCompassCTRL3()
{
    unsigned char ctrl3 = 0; // continuous conversion mode

    return (m_settings->HALWrite(m_magSlaveAddr, LIS3MDL_CTRL_REG3, ctrl3, "Failed to set LIS3MDL compass CTRL3"));
}

bool RTIMULSM6DS33::setCompassCTRL4()
{
    unsigned char ctrl4 =
        (m_settings->m_LIS3MDLCompassZMode << 2);

    if (m_settings->m_LIS3MDLCompassZMode < 0 || m_settings->m_LIS3MDLCompassZMode > 3) {
        HAL_ERROR1("Illegal LIS3MDL Z operating mode code %d\n", m_settings->m_LIS3MDLCompassZMode);
        return false;
    }

    return (m_settings->HALWrite(m_magSlaveAddr, LIS3MDL_CTRL_REG4, ctrl4, "Failed to set LIS3MDL compass CTRL4"));
}

int RTIMULSM6DS33::IMUGetPollInterval()
{
    if (m_sampleRate > 400) return 1;
    return (400 / m_sampleRate);
}

bool RTIMULSM6DS33::IMURead()
{
    unsigned char status, cstatus;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_STATUS_REG, 1, &status, "Failed to read LSM6DS33 status"))
        return false;

    if (!m_settings->HALRead(m_magSlaveAddr, LIS3MDL_STATUS_REG, 1, &cstatus, "Failed to read LIS3MDL status"))
        return false;

    if ((status & 0x3) == 0 || (cstatus & 0x3) == 0)
        return false;

    for (int i = 0; i<6; i++){
        if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_OUT_X_L_G + i, 1, &gyroData[i], "Failed to read LSM6DS33 gyro data"))
            return false;

        if (!m_settings->HALRead(m_accelGyroSlaveAddr, LSM6DS33_OUT_X_L_XL + i, 1, &accelData[i], "Failed to read LSM6DS33 accel data"))
            return false;

        if (!m_settings->HALRead(m_magSlaveAddr, LIS3MDL_OUT_X_L + i, 1, &compassData[i], "Failed to read LIS3MDL compass data"))
            return false;
    }

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes and correct for bias

    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    m_imuData.compass.setY(-m_imuData.compass.y());
    m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter

    updateFusion();

    return true;
}

