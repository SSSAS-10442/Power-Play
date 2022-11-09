package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.Gamepad;

public class LastKey {

    Gamepad gamepad;

    boolean a = false;
    boolean b = false;
    boolean x = false;
    boolean y = false;

    boolean lb = false;
    boolean rb = false;

    double lt = 0;
    double rt = 0;

    double lx = 0;
    double ly = 0;
    double rx = 0;
    double ry = 0;

    boolean dpad_up = false;
    boolean dpad_left = false;
    boolean dpad_right = false;
    boolean dpad_down = false;

    public LastKey(Gamepad gamepad) {
        this.gamepad = gamepad;
        update();
    }

    void update() {

        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;

        lb = gamepad.left_bumper;
        rb = gamepad.right_bumper;

        lt = gamepad.left_trigger;
        rt = gamepad.right_trigger;

        lx = gamepad.left_stick_x;
        ly = gamepad.left_stick_y;
        rx = gamepad.right_stick_x;
        ry = gamepad.right_stick_y;

        dpad_up = gamepad.dpad_up;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        dpad_down = gamepad.dpad_down;

    }

}
