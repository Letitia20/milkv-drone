#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const char* path) {
    std::ifstream in(path);
    if (!in) {
        return {};
    }

    std::ostringstream contents;
    contents << in.rdbuf();
    return contents.str();
}

bool expectContains(const std::string& text,
                    const std::string& needle,
                    const std::string& label) {
    if (text.find(needle) != std::string::npos) {
        return true;
    }

    std::cerr << "missing " << label << ": " << needle << "\n";
    return false;
}

bool expectNotContains(const std::string& text,
                       const std::string& needle,
                       const std::string& label) {
    if (text.find(needle) == std::string::npos) {
        return true;
    }

    std::cerr << "unexpected " << label << ": " << needle << "\n";
    return false;
}

}  // namespace

int main() {
    const std::string script = readFile("install_autostart.sh");
    if (script.empty()) {
        std::cerr << "install_autostart.sh is missing or empty\n";
        return 1;
    }

    bool ok = true;
    ok = expectContains(script, "AUTO_SH=\"/mnt/system/auto.sh\"",
                        "targets Milk-V startup script") && ok;
    ok = expectContains(script, "APP=\"/root/milkv_drone\"",
                        "runs installed controller binary") && ok;
    ok = expectContains(script, "IBUS_DEVICE=\"${IBUS_DEVICE:-/dev/ttyS1}\"",
                        "defaults to iBUS UART") && ok;
    ok = expectContains(script, "BAUDRATE=\"${BAUDRATE:-115200}\"",
                        "defaults to 115200 baud") && ok;
    ok = expectContains(script, "\"${APP}\" \"${IBUS_DEVICE}\" \"${BAUDRATE}\"",
                        "starts controller with device and baud") && ok;
    ok = expectNotContains(script, "ESTOP_GPIO",
                           "hardware emergency-stop GPIO config") && ok;
    ok = expectNotContains(script, "--estop-gpio",
                           "hardware emergency-stop command-line argument") && ok;
    ok = expectContains(script, "chmod +x \"${AUTO_SH}\"",
                        "marks auto.sh executable") && ok;

    if (!ok) {
        return 1;
    }

    std::cout << "autostart_script_test passed\n";
    return 0;
}
