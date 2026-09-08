// 测试目标：验证根目录 ESC 台架测试程序的电机映射和 PWM 脉宽换算。
#define ESC_PWM_UNIT_TEST
#include "../main.cpp"

#include <cassert>
#include <string>

int main() {
    const auto& escs = esc_configs();

    assert(escs.size() == 4);
    assert(escs[0].esc_id == 1 && std::string(escs[0].gpio) == "GP4" && escs[0].pin == 6 &&
           escs[0].pwm_channel == 5 && escs[0].chip_base == 4 && escs[0].local_channel == 1);
    assert(escs[1].esc_id == 2 && std::string(escs[1].gpio) == "GP5" && escs[1].pin == 7 &&
           escs[1].pwm_channel == 6 && escs[1].chip_base == 4 && escs[1].local_channel == 2);
    assert(escs[2].esc_id == 3 && std::string(escs[2].gpio) == "GP12" && escs[2].pin == 16 &&
           escs[2].pwm_channel == 4 && escs[2].chip_base == 4 && escs[2].local_channel == 0);
    assert(escs[3].esc_id == 4 && std::string(escs[3].gpio) == "GP6" && escs[3].pin == 9 &&
           escs[3].pwm_channel == 9 && escs[3].chip_base == 8 && escs[3].local_channel == 1);

    assert(clamp_esc_pulse_us(900) == 1000);
    assert(clamp_esc_pulse_us(1500) == 1500);
    assert(clamp_esc_pulse_us(2500) == 2000);
    assert(clamp_test_pulse_us(1250) == 1200);
    assert(us_to_ns(1150) == 1150000);

    return 0;
}
