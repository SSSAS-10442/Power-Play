package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Arm;

@TeleOp(name="ArmTest",group="Power-Play-Extra")
public class ArmTest extends OpMode {

    private DcMotorEx arm;

    public void init() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
    }

    public void loop() {

        if (gamepad1.a) {
            arm.setPower(1);
        } else {
            arm.setPower(0);
        }

        telemetry.addLine("A: Arm");
        telemetry.update();

    }

}
