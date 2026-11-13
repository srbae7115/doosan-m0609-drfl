// SCHUNK Co-act EGP-C 40-N-N-M1013 (1359018) on a Doosan tool flange: open/close via flange DO.
// No robot motion. Requests control authority (force) only to switch the flange outputs.
//
// Wiring of the M1013 variant (SCHUNK manual 389336 v19, connection version 6):
//   M8 pin 2 -> flange DO1 = Close gripper     M8 pin 1 -> flange DI1 = Sensor 1 (=1 at closed end)
//   M8 pin 3 -> flange DO2 = Open gripper      M8 pin 7 -> flange DI2 = Sensor 2 (=1 at open end)
// Truth table (DO2 open, DO1 close): 0/0 drive off, 1/0 open, 0/1 close, 1/1 acknowledge error.
// Keep >= 15 ms between two commands (we drop both outputs and wait 50 ms first).
// Gripping force is set only by the rotary switch on the gripper (25/50/75/100 %).
//
// usage: ./tests/egp_gripper_test [IP]
#ifndef DRCF_VERSION
#define DRCF_VERSION 2
#endif
#include "DRFLEx.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
using namespace DRAFramework;

constexpr GPIO_TOOL_DIGITAL_INDEX DO_CLOSE = GPIO_TOOL_DIGITAL_INDEX_1;
constexpr GPIO_TOOL_DIGITAL_INDEX DO_OPEN = GPIO_TOOL_DIGITAL_INDEX_2;
constexpr GPIO_TOOL_DIGITAL_INDEX DI_SENSOR1 = GPIO_TOOL_DIGITAL_INDEX_1;
constexpr GPIO_TOOL_DIGITAL_INDEX DI_SENSOR2 = GPIO_TOOL_DIGITAL_INDEX_2;

static CDRFLEx robot;
static std::atomic<bool> g_access{false};

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

static bool set_outputs(bool open, bool close) {
    // Always pass through 0/0 so two commands never follow each other directly.
    bool ok = robot.set_tool_digital_output(DO_OPEN, false) &&
              robot.set_tool_digital_output(DO_CLOSE, false);
    sleep_ms(50);
    if (open) ok = robot.set_tool_digital_output(DO_OPEN, true) && ok;
    if (close) ok = robot.set_tool_digital_output(DO_CLOSE, true) && ok;
    return ok;
}

static void print_io(const char* tag) {
    std::cout << tag << "  DO1(close)=" << robot.get_tool_digital_output(DO_CLOSE)
              << " DO2(open)=" << robot.get_tool_digital_output(DO_OPEN)
              << "  DI1(sensor1)=" << robot.get_tool_digital_input(DI_SENSOR1)
              << " DI2(sensor2)=" << robot.get_tool_digital_input(DI_SENSOR2) << std::endl;
}

int main(int argc, char** argv) {
    std::string ip = argc > 1 ? argv[1] : "192.168.137.100";
    robot.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL a) {
        if (a == MONITORING_ACCESS_CONTROL_GRANT) g_access = true;
        if (a == MONITORING_ACCESS_CONTROL_LOSS) g_access = false;
    });
    if (!robot.open_connection(ip)) { std::cerr << "[ERR] open_connection failed: " << ip << "\n"; return 1; }
    robot.setup_monitoring_version(1);
    sleep_ms(1000);
    for (int i = 0; i < 10 && !g_access; ++i) {
        robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        sleep_ms(1000);
    }
    if (!g_access) { std::cerr << "[ERR] no control authority\n"; robot.close_connection(); return 1; }

    print_io("[now]");
    std::cout << "\n o = open   c = close   x = drive off (0/0)   a = acknowledge error (1/1, 0.5 s)\n"
                 " i = read I/O   q = quit (outputs stay as they are)\n"
                 "Keep fingers away from the jaws. Enter a key + Enter.\n";
    std::string line;
    while (std::cout << "> " << std::flush, std::getline(std::cin, line)) {
        if (line.empty()) continue;
        bool ok = true;
        switch (line[0]) {
            case 'o': ok = set_outputs(true, false); break;
            case 'c': ok = set_outputs(false, true); break;
            case 'x': ok = set_outputs(false, false); break;
            case 'a': ok = set_outputs(true, true); sleep_ms(500); ok = set_outputs(false, false) && ok; break;
            case 'i': break;
            case 'q': robot.close_connection(); return 0;
            default: std::cout << "?\n"; continue;
        }
        if (!ok) std::cerr << "[ERR] set_tool_digital_output rejected (control authority / robot mode?)\n";
        sleep_ms(600);  // EGP-C 40 open/close takes ~0.2-0.5 s
        print_io("     ");
    }
    robot.close_connection();
    return 0;
}
