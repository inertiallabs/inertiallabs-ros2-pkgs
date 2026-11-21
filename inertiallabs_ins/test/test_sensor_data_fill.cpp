#include <gtest/gtest.h>

#include <inertiallabs_msgs/msg/sensor_data.hpp>

#include "sensor_data_fill.h"

TEST(SensorDataFill, EachFieldComesFromItsOwnSource)
{
    IL::INSDataStruct data{};
    data.Mag[0] = 1.0; data.Mag[1] = 2.0; data.Mag[2] = 3.0;
    data.Acc[0] = 4.0; data.Acc[1] = 5.0; data.Acc[2] = 6.0;
    data.Gyro[0] = 7.0; data.Gyro[1] = 8.0; data.Gyro[2] = 9.0;
    data.pBar = 101325.0;  // pressure
    data.hBar = 123.5;     // barometric height

    inertiallabs_msgs::msg::SensorData msg;
    fillSensorData(data, msg);

    EXPECT_DOUBLE_EQ(msg.mag.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.mag.y, 2.0);
    EXPECT_DOUBLE_EQ(msg.mag.z, 3.0);
    EXPECT_DOUBLE_EQ(msg.accel.z, 6.0);
    EXPECT_DOUBLE_EQ(msg.gyro.z, 9.0);
    EXPECT_FLOAT_EQ(msg.pressure, 101325.0f);
    EXPECT_FLOAT_EQ(msg.barometric_height, 123.5f);
}
