#pragma once

#include <string>

namespace drone {

// 发送给 HC-05 的遥测快照。
// function_sensor_temp_c 使用 MPU6050 温度作为一路功能传感器数据。
struct BluetoothTelemetrySnapshot {
    double roll_deg {0.0};
    double pitch_deg {0.0};
    double yaw_deg {0.0};
    int battery_mv {0};
    bool low_voltage {false};
    bool armed {false};
    double function_sensor_temp_c {0.0};
};

// 蓝牙串口输出格式：
// TEL,roll,pitch,yaw,battery_mv,low_voltage,armed,function_sensor_temp_c
std::string encodeBluetoothTelemetry(const BluetoothTelemetrySnapshot& snapshot);

}  // namespace drone
