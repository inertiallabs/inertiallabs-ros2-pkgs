#pragma once

#include "dataStructures.h"

// Copies the sensor fields of an INS data packet into a SensorData message.
template <typename SensorDataMsg>
void fillSensorData(const IL::INSDataStruct& data, SensorDataMsg& msg)
{
    msg.mag.x = data.Mag[0];
    msg.mag.y = data.Mag[1];
    msg.mag.z = data.Mag[2];
    msg.accel.x = data.Acc[0];
    msg.accel.y = data.Acc[1];
    msg.accel.z = data.Acc[2];
    msg.gyro.x = data.Gyro[0];
    msg.gyro.y = data.Gyro[1];
    msg.gyro.z = data.Gyro[2];
    msg.temp = data.Temp;
    msg.vinp = data.VSup;
    msg.pressure = data.pBar;
    msg.barometric_height = data.hBar;
}
