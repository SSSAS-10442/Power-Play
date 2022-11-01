package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    DcMotorEx motor;

    public Arm(DcMotorEx arm) {
        this.motor = arm;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void scoringL() {
        motor.setTargetPosition(Main.ARM_SCORING_L);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringM() {
        motor.setTargetPosition(Main.ARM_SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringS() {
        motor.setTargetPosition(Main.ARM_SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringG() {
        motor.setTargetPosition(Main.ARM_SCORING_GROUND);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
