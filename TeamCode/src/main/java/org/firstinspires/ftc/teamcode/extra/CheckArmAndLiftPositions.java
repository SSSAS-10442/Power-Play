package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Arm;
import org.firstinspires.ftc.teamcode.main.Lift;

@Disabled
@TeleOp(name="CheckArmAndLiftPositions", group="Power-Play-Extra")
public class CheckArmAndLiftPositions extends OpMode {

    Arm arm = null;
    Lift lift = null;

    @Override
    public void init() {
        arm = new Arm(hardwareMap.get(DcMotorEx.class, "arm"));
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "lift"));
    }

    @Override
    public void loop() {

        arm.motor.setVelocity(1);
        lift.motor.setVelocity(1);

        if (gamepad1.a) {
            arm.motor.setTargetPosition(arm.motor.getCurrentPosition() - 1);
        } else if (gamepad1.b) {
            arm.motor.setTargetPosition(arm.motor.getCurrentPosition() + 1);
        }
        if (gamepad1.x) {
            lift.motor.setTargetPosition(lift.motor.getCurrentPosition() - 1);
        } else if (gamepad1.y) {
            lift.motor.setTargetPosition(lift.motor.getCurrentPosition() + 1);
        }

        telemetry.addLine("Arm: " + arm.motor.getCurrentPosition());
        telemetry.addLine("Lift: " + lift.motor.getCurrentPosition());
        telemetry.update();

    }
}
