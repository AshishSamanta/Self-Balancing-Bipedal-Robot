#ifndef DUALSHOCK_H
#define DUALSHOCK_H

#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int on_off_state = -1;
int body_height_cmd = 200; 
bool is_waiting = false;
bool toggled = false;
unsigned long press_start_time = 0;

struct JOY_CMD {
    int on_off_state;
    int body_height_cmd;
    float left_joy_cmd;
    float right_joy_cmd;
};

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller connected at index=%d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
    Serial.println("No empty slot for controller!");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            return;
        }
    }
    Serial.println("Controller disconnected but not found");
}

JOY_CMD processGamepad(ControllerPtr ctl) {
    static unsigned long press_start_time = 0;
    static bool is_waiting = false;
    static bool toggled = false;
    static int on_off_state = -1;  // 1 = active, -1 = inactive
    static int body_height_cmd = 300;  // persistent between calls

    // Toggle on/off when button combo 0x0030 is held for 2s
    if (ctl->buttons() == 0x0020) {
        if (!is_waiting) {
            press_start_time = millis();
            is_waiting = true;
            toggled = false;
        } else {
            if ((millis() - press_start_time >= 2000) && !toggled) {
                on_off_state *= -1;
                toggled = true;
            }
        }
    } else {
        is_waiting = false;
        toggled = false;
    }

    // D-Pad control for height
    uint8_t dpad = ctl->dpad();
    if (dpad == 0x01) {  // UP
        body_height_cmd += 5;
    } else if (dpad == 0x02) {  // DOWN
        body_height_cmd -= 5;
    }

    // Clamp body height between 170 and 300
    if (body_height_cmd > 320) body_height_cmd = 320;
    if (body_height_cmd < 170) body_height_cmd = 170;

    // Return command
    if (on_off_state == -1) {
        return JOY_CMD{on_off_state, body_height_cmd, 0.0, 0.0};
    } else {
        return JOY_CMD{on_off_state, body_height_cmd, ctl->axisY(), ctl->axisRX()};
    }
}

JOY_CMD processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            return processGamepad(myController);
        }
    }
    return JOY_CMD{on_off_state, 300, 0.0, 0.0}; // fallback if no data
}

void setupJOY() {
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); // remove this if you don't want to re-pair every reboot
    BP32.enableVirtualDevice(false);
}

JOY_CMD UpdateJoyloop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        return processControllers();
    } else {
        return JOY_CMD{on_off_state, 300, 0.0, 0.0};
    }
}

#endif
