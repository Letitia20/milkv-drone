#define ESC_PWM_UNIT_TEST
#include "../main.cpp"

#include <cassert>
#include <string>

int main() {
    const auto& escs = esc_configs();

    assert(escs.size() == 4);
    assert(escs[0].esc_id == 1 && std::string(escs[0].gpio) == "GP4" && escs[0].pin == 6 && escs[0].pwm_channel == 5);
    assert(escs[1].esc_id == 2 && std::string(escs[1].gpio) == "GP5" && escs[1].pin == 7 && escs[1].pwm_channel == 6);
    assert(escs[2].esc_id == 3 && std::string(escs[2].gpio) == "GP12" && escs[2].pin == 16 && escs[2].pwm_channel == 4);
    assert(escs[3].esc_id == 4 && std::string(escs[3].gpio) == "GP2" && escs[3].pin == 4 && escs[3].pwm_channel == 7);

    assert(clamp_esc_pulse_us(900) == 1000);
    assert(clamp_esc_pulse_us(1500) == 1500);
    assert(clamp_esc_pulse_us(2500) == 2000);
    assert(clamp_test_pulse_us(1250) == 1200);
    assert(us_to_ns(1150) == 1150000);

    return 0;
}
