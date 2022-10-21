package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {

    DcMotorEx motor;

    Arm(DcMotorEx arm) {
        this.motor = arm;
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void scoringL() {
        motor.setTargetPosition(Main.SCORING_L);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringM() {
        motor.setTargetPosition(Main.SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringS() {
        motor.setTargetPosition(Main.SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void scoringG() {
        motor.setTargetPosition(Main.SCORING_GROUND);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
