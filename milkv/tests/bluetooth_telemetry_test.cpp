// 测试目标：验证 HC-05 蓝牙遥测行格式为 TEL,roll,pitch,yaw,battery,low_voltage,armed,temp。
#include "bluetooth_telemetry.hpp"

#include <iostream>
#include <string>

int main() {
    drone::BluetoothTelemetrySnapshot snapshot;
    snapshot.roll_deg = 1.234;
    snapshot.pitch_deg = -2.345;
    snapshot.yaw_deg = 30.0;
    snapshot.battery_mv = 9500;
    snapshot.low_voltage = true;
    snapshot.armed = false;
    snapshot.function_sensor_temp_c = 28.75;

    const std::string line = drone::encodeBluetoothTelemetry(snapshot);
    const std::string expected = "TEL,1.23,-2.35,30.00,9500,1,0,28.75";
    if (line != expected) {
        std::cerr << "telemetry line expected '" << expected << "' got '"
                  << line << "'\n";
        return 1;
    }

    return 0;
}
