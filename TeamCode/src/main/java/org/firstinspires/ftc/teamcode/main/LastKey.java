package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.Gamepad;

public class LastKey {

    Gamepad gamepad;

    public boolean a = false;
    public boolean b = false;
    public boolean x = false;
    public boolean y = false;

    public boolean lb = false;
    public boolean rb = false;

    public double lt = 0;
    public double rt = 0;

    public double lx = 0;
    public double ly = 0;
    public double rx = 0;
    public double ry = 0;

    public boolean dpad_up = false;
    public boolean dpad_left = false;
    public boolean dpad_right = false;
    public boolean dpad_down = false;

    public LastKey(Gamepad gamepad) {
        this.gamepad = gamepad;
        update();
    }

    public void update() {

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
