package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    public DcMotorEx motor;
    static final int allowance = 1;

    public Lift(DcMotorEx lift) {
        this.motor = lift;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void go() {
        go(500);
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
        motor.setTargetPosition(Main.LIFT_SCORING_L);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void scoringM() {
        motor.setTargetPosition(Main.LIFT_SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void scoringS() {
        motor.setTargetPosition(Main.LIFT_SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
        checkShouldStop();
    }

    void scoringG() {
        motor.setTargetPosition(Main.LIFT_SCORING_GROUND);
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

