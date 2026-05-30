// sh1106_notify.cpp
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <px4_platform_common/getopt.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>           // <- use this, not vehicle_gps_position


#include <uORB/Subscription.hpp>               // polling subscriptions
#include <string.h>
#include <stdio.h>
#include <stdlib.h>                            // atoi, strtol

#include "sh1106.hpp"

extern "C" __EXPORT int sh1106_notify_main(int argc, char *argv[]);
class Sh1106Notify
    : public ModuleBase<Sh1106Notify>,
      public px4::ScheduledWorkItem
{
public:
    Sh1106Notify(int bus, int addr);
    ~Sh1106Notify() override;

    static int task_spawn(int argc, char *argv[]);                          // <-- [] not bare pointer
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void Run() override;

private:
    SH1106 _oled_display;

    // Use simple polling subscriptions (no callback registration needed)
    uORB::Subscription _telemetry_sub{ORB_ID(telemetry_status)};
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _radio_status_sub{ORB_ID(radio_status)};
    uORB::Subscription _battery_sub{ORB_ID(battery_status)};
    uORB::Subscription _gps_sub{ORB_ID(sensor_gps)};

    battery_status_s _battery{};
     sensor_gps_s     _gps{};
    telemetry_status_s _telemetry_status{};
    vehicle_status_s   _vehicle_status{};
    radio_status_s     _radio_status{};
};
static const char *nav_state_str(uint8_t nav_state)
{
    using VS = vehicle_status_s;
    switch (nav_state) {
    case VS::NAVIGATION_STATE_MANUAL:            return "MANUAL";
    case VS::NAVIGATION_STATE_ALTCTL:            return "ALTCTL";
    case VS::NAVIGATION_STATE_POSCTL:            return "POSCTL";
    case VS::NAVIGATION_STATE_AUTO_MISSION:      return "MISSION";
    case VS::NAVIGATION_STATE_AUTO_LOITER:       return "HOLD";
    case VS::NAVIGATION_STATE_AUTO_RTL:          return "RETURN";
    case VS::NAVIGATION_STATE_ACRO:              return "ACRO";
    case VS::NAVIGATION_STATE_OFFBOARD:          return "OFFBD";
    case VS::NAVIGATION_STATE_STAB:              return "STABLE";
    case VS::NAVIGATION_STATE_AUTO_TAKEOFF:      return "TKOFF";
    case VS::NAVIGATION_STATE_AUTO_LAND:         return "LAND";
    case VS::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:return "FOLLOW";
    default:                                     return "UNKNOWN";
    }
}

Sh1106Notify::Sh1106Notify(int bus, int addr)
: px4::ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
  _oled_display(bus, addr)
{
    PX4_INFO("starting (bus=%d addr=0x%02x)", bus, addr);

    const int ret = _oled_display.init();
    if (ret != PX4_OK) {
        PX4_ERR("SH1106 init failed (%d) on bus=%d addr=0x%02x", ret, bus, addr);
    } else {
        PX4_INFO("SH1106 init OK on bus=%d addr=0x%02x", bus, addr);
        ScheduleNow();
    }
}

Sh1106Notify::~Sh1106Notify()
{
    _oled_display.clear();
}
void Sh1106Notify::Run()
{
    if (should_exit()) return;

    // update topics
    (void)_telemetry_sub.update(&_telemetry_status);
    (void)_vehicle_status_sub.update(&_vehicle_status);
    (void)_radio_status_sub.update(&_radio_status);
    (void)_battery_sub.update(&_battery);
    (void)_gps_sub.update(&_gps);

    // 21 chars + NUL
    char line1[22]{}, line2[22]{}, line3[22]{}, line4[22]{},line5[22]{};

    // ---- Flight mode / arming ----
    const char *mode  = nav_state_str(_vehicle_status.nav_state);
    const char *armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? "ARM" : "DISARM";
    snprintf(line1, sizeof(line1), "%s | %s", mode, armed);

    // ---- Battery ----
    float v = _battery.voltage_v;
    int rem_pct = -1;
    if (_battery.remaining > 0.f && _battery.remaining <= 1.f) {
        rem_pct = int(_battery.remaining * 100.f + 0.5f);
    }
    if (PX4_ISFINITE(v)) {
        if (rem_pct >= 0) snprintf(line2, sizeof(line2), "Batt: %.2fV %d%%", (double)v, rem_pct);
        else              snprintf(line2, sizeof(line2), "Batt: %.2fV", (double)v);
    } else {
        snprintf(line2, sizeof(line2), "Batt: ---");
    }

    // ---- GPS ----
    const char *fix =
        (_gps.fix_type >= 6) ? "RTKFIX" :
        (_gps.fix_type == 5) ? "RTKFLT" :
        (_gps.fix_type == 4) ? "DGPS" :
        (_gps.fix_type == 3) ? "3D" :
        (_gps.fix_type == 2) ? "2D" :
        (_gps.fix_type == 1) ? "NOFIX" : "NOGPS";
    snprintf(line3, sizeof(line3), "GPS: %s Sats:%u", fix, (unsigned)_gps.satellites_used);

    // ---- Link (radio) ----
    // Use radio_status fields that exist on your branch
    int rssi_local  = (int)_radio_status.rssi;         // 0..100 or -1 if N/A
    int rssi_remote = (int)_radio_status.remote_rssi;  // 0..100 or -1 if N/A

    if (rssi_local >= 0 && rssi_remote >= 0) {
        snprintf(line4, sizeof(line4), "Link: RSSI:%d REM:%d", rssi_local, rssi_remote);
    } else if (rssi_local >= 0) {
        snprintf(line4, sizeof(line4), "Link: RSSI:%d", rssi_local);
    } else {
        snprintf(line4, sizeof(line4), "Link: ---");
    }
    static char prev_line1[22]{}, prev_line2[22]{}, prev_line3[22]{}, prev_line4[22]{},prev_line5[22]{};
// Show TX buffer percent and RX error count (fields that are present)
    snprintf(line5, sizeof(line5), "TX:%d%%  ERR:%u",
         (int)_radio_status.txbuf, (unsigned)_radio_status.rxerrors);
    if (strcmp(line1, prev_line1) != 0) {
        _oled_display.print_line(0, line1);
        strcpy(prev_line1, line1);
    }
    if (strcmp(line2, prev_line2) != 0) {
        _oled_display.print_line(1, line2);
        strcpy(prev_line2, line2);
    }
    if (strcmp(line3, prev_line3) != 0) {
        _oled_display.print_line(0, line3);
        strcpy(prev_line3, line3);
    }
    if (strcmp(line4, prev_line4) != 0) {
        _oled_display.print_line(1, line4);
        strcpy(prev_line4, line4);
    }
        if (strcmp(line5, prev_line5) != 0) {
        _oled_display.print_line(1, line5);
        strcpy(prev_line5, line5);
    }
    // draw
    //_oled_display.clear();
    _oled_display.print_line(0, line1);
    _oled_display.print_line(1, line2);
    _oled_display.print_line(2, line3);
    _oled_display.print_line(3, line4);
    _oled_display.print_line(4, line5);

    ScheduleDelayed(500000); // T=500ms, aka 2Hz
}


int Sh1106Notify::task_spawn(int argc, char *argv[])
{
    int bus  = 1;        // defaults you want
    int addr = 0x3c;

    // parse: sh1106_notify start -b <bus> -a <addr>
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "-b") == 0 && (i + 1) < argc) {
            bus = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "-a") == 0 && (i + 1) < argc) {
            addr = static_cast<int>(std::strtol(argv[++i], nullptr, 0)); // supports 0x3c or 60
        } else if (std::strcmp(argv[i], "start") == 0) {
            // ignore
        } else {
            return print_usage("bad argument");
        }
    }

    auto *inst = new Sh1106Notify(bus, addr);
    if (!inst) {
        PX4_ERR("alloc failed");
        return PX4_ERROR;
    }

    // Atomically publish the instance pointer
    _object.store(inst);

    // If you're running on a work queue, kick it off now
    inst->ScheduleNow();

    // Let ModuleBase know this is a WQ-based module
    _task_id = task_id_is_work_queue;

    return PX4_OK;
}
int Sh1106Notify::custom_command(int argc, char *argv[])
{
    return print_usage("custom command is not supported");
}



int Sh1106Notify::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Driver for SH1106 OLED display used for notifications.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sh1106_notify", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('b', 2, 1, 4, "I2C bus", true);
    PRINT_MODULE_USAGE_PARAM_INT('a', 0x3c, 0, 127, "I2C address", true);
    return 0;
}

extern "C" __EXPORT int sh1106_notify_main(int argc, char *argv[])
{
    return Sh1106Notify::main(argc, argv);
}
