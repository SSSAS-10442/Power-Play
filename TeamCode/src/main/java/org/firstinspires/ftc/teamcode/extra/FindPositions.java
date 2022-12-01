package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Arm;
import org.firstinspires.ftc.teamcode.main.LastKey;
import org.firstinspires.ftc.teamcode.main.Lift;
import org.firstinspires.ftc.teamcode.main.Main;
import org.firstinspires.ftc.teamcode.main.MotorsEx;

//@Disabled
@TeleOp(name="FindPositions", group="Power-Play-Extra")
public class FindPositions extends OpMode {

    DcMotorEx arm = null;
    DcMotorEx lift = null;

    LastKey last = null;

    @Override
    public void init() {

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        lift.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            arm.setTargetPosition(arm.getTargetPosition() + 5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(2000);
        } else if (gamepad1.y) {
            arm.setTargetPosition(arm.getTargetPosition() - 5);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setVelocity(2000);
        }

        if (gamepad1.b) {
            lift.setTargetPosition(lift.getTargetPosition() + 5);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(2000);
        } else if (gamepad1.a) {
            lift.setTargetPosition(lift.getTargetPosition() - 5);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setVelocity(2000);
        }

        if (gamepad1.left_trigger > 0.2) {
            arm.setVelocity(0);
            lift.setVelocity(0);
        }

        telemetry.addLine("Arm: " + arm.getTargetPosition());
        telemetry.addLine("Lift: " + lift.getTargetPosition());
        telemetry.addLine("X: Arm +50");
        telemetry.addLine("Y: Arm -50");
        telemetry.addLine("B: Lift +50");
        telemetry.addLine("A: Lift -50");
        telemetry.update();

    }
}
