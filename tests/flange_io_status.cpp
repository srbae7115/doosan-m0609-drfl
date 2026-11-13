// Read-only: flange (tool) DI/DO state. No control request, no outputs.
#ifndef DRCF_VERSION
#define DRCF_VERSION 2
#endif
#include "DRFLEx.h"
#include <chrono>
#include <iostream>
#include <thread>
using namespace DRAFramework;
int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "192.168.137.100";
    CDRFLEx robot;
    if (!robot.open_connection(ip)) { std::cerr << "connect failed\n"; return 1; }
    robot.setup_monitoring_version(1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    SYSTEM_VERSION v = {'\0'};
    robot.get_system_version(&v);
    std::cout << "Flange board: " << v._szFlangeBoard << "\n";
    for (int k = 0; k < 3; ++k) {
        std::cout << "tool DI 1-6:";
        for (int i = 0; i < 6; ++i) std::cout << " " << robot.get_tool_digital_input((GPIO_TOOL_DIGITAL_INDEX)i);
        std::cout << "   tool DO 1-6:";
        for (int i = 0; i < 6; ++i) std::cout << " " << robot.get_tool_digital_output((GPIO_TOOL_DIGITAL_INDEX)i);
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    robot.close_connection();
}
