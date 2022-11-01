package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    DcMotorEx motor;
    static final int allowance = 5;

    public Arm(DcMotorEx arm) {
        this.motor = arm;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void go() {
        go(1000);
    }

    void go(int velocity) {
        motor.setVelocity(velocity);
    }

    void stop() {
        motor.setVelocity(0);
    }
    
    void checkShouldStop() {
        if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < allowance) {
            stop();
        }
    }

    void scoringL() {
        motor.setTargetPosition(Main.ARM_SCORING_L);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void scoringM() {
        motor.setTargetPosition(Main.ARM_SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void scoringS() {
        motor.setTargetPosition(Main.ARM_SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void scoringG() {
        motor.setTargetPosition(Main.ARM_SCORING_GROUND);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

}
