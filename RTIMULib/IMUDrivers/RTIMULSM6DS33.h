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


#ifndef _RTIMULSM6DS33_H
#define	_RTIMULSM6DS33_H

#include "RTIMU.h"

class RTIMULSM6DS33 : public RTIMU
{
public:
    RTIMULSM6DS33(RTIMUSettings *settings);
    ~RTIMULSM6DS33();

    virtual const char *IMUName() { return "LSM6DS33 + LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DS33; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setAccelCTRL1();
    bool setGyroCTRL2();
    bool setCompassCTRL1();
    bool setCompassCTRL2();
    bool setCompassCTRL3();
    bool setCompassCTRL4();

    unsigned char m_accelGyroSlaveAddr;                     // I2C address of accel andgyro
    unsigned char m_magSlaveAddr;                           // I2C address of mag

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

#endif // _RTIMULSM6DS33_H
