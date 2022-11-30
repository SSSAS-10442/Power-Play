package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp(name="Claw-Control",group="Power-Play-Extra")
public class ClawControl extends OpMode {

    private Servo claw = null;
    private double clawPos = 0.5;
    private boolean aLastFrame = false;
    private boolean bLastFrame = false;

    @Override
    public void init() {

        claw = hardwareMap.get(Servo.class, "claw");

    }

    @Override
    public void loop() {

        if (gamepad1.a && !aLastFrame) {
            clawPos += 0.05;
        }
        if (gamepad1.b && !bLastFrame) {
            clawPos -= 0.05;
        }

        claw.setPosition(clawPos);

        aLastFrame = gamepad1.a;
        bLastFrame = gamepad1.b;

    }
}
