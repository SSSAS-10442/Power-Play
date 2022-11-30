package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Arm;
import org.firstinspires.ftc.teamcode.main.LastKey;
import org.firstinspires.ftc.teamcode.main.Lift;
import org.firstinspires.ftc.teamcode.main.Main;
import org.firstinspires.ftc.teamcode.main.MotorsEx;

@TeleOp(name="FindPositions", group="Power-Play-Extra")
public class FindPositions extends OpMode {

    MotorsEx motors = null;
    DcMotorEx arm = null;
    DcMotorEx lift = null;

    LastKey last = null;

    @Override
    public void init() {

        motors = new MotorsEx(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "rightFront"),
                hardwareMap.get(DcMotorEx.class, "leftBack"),
                hardwareMap.get(DcMotorEx.class, "rightBack")
        );

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        last = new LastKey(gamepad1);

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
            arm.setTargetPosition(arm.getTargetPosition() + 1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.y) {
            arm.setTargetPosition(arm.getTargetPosition() - 1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.b) {
            lift.setTargetPosition(lift.getTargetPosition() + 1);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad1.a) {
            lift.setTargetPosition(lift.getTargetPosition() - 1);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (Math.abs(arm.getTargetPosition() - arm.getCurrentPosition()) <= 10) {
            arm.setVelocity(0);
        } else {
            arm.setVelocity(300);
        }
        if (Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) <= 10) {
            lift.setVelocity(0);
        } else {
            lift.setVelocity(300);
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
