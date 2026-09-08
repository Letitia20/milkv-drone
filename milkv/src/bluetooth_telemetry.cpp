#include "bluetooth_telemetry.hpp"

#include <iomanip>
#include <sstream>

namespace drone {

std::string encodeBluetoothTelemetry(const BluetoothTelemetrySnapshot& snapshot) {
    std::ostringstream out;
    // 保留两位小数，手机蓝牙串口助手可直接可视化/记录。
    out << std::fixed << std::setprecision(2)
        << "TEL,"
        << snapshot.roll_deg << ','
        << snapshot.pitch_deg << ','
        << snapshot.yaw_deg << ','
        << snapshot.battery_mv << ','
        << (snapshot.low_voltage ? 1 : 0) << ','
        << (snapshot.armed ? 1 : 0) << ','
        << snapshot.function_sensor_temp_c;
    return out.str();
}

}  // namespace drone
