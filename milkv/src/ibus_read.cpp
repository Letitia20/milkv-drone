#include "ibus_receiver.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running {true};

void handleSignal(int) {
    g_running = false;
}

void printChannels(const drone::IBusChannels& channels) {
    for (std::size_t i = 0; i < channels.ch.size(); ++i) {
        if (i > 0) {
            std::cout << ' ';
        }
        std::cout << "CH" << (i + 1) << '=' << channels.ch[i];
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = (argc >= 2) ? argv[1] : "/dev/ttyS1";

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    drone::IBusReceiver receiver;
    if (!receiver.open(device.c_str())) {
        std::cerr << "Failed to open iBUS on " << device << ": " << receiver.lastError() << "\n";
        return 1;
    }

    std::cerr << "Reading FlySky iBUS from " << device << " at 115200 8N1 raw\n";

    drone::IBusChannels channels;
    while (g_running) {
        if (receiver.readFrame(channels)) {
            printChannels(channels);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    receiver.close();
    return 0;
}
