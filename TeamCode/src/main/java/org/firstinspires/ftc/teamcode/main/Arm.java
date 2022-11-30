package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    public DcMotorEx motor;
    static final int allowance = 1;

    public Arm(DcMotorEx arm) {
        this.motor = arm;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        checkShouldStop();
    }

    void scoringM() {
        motor.setTargetPosition(Main.ARM_SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void scoringS() {
        motor.setTargetPosition(Main.ARM_SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void scoringG() {
        motor.setTargetPosition(Main.ARM_SCORING_GROUND);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

}
