package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {

    public DcMotorEx motor;
    static final int allowance = 5;

    public Lift(DcMotorEx lift) {
        this.motor = lift;
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void go() {
        go(1000);
    }

    void go(int velocity) {
        motor.setVelocity(velocity);
    }

    void stop() {
        motor.setVelocity(1);
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
    }

    void scoringM() {
        motor.setTargetPosition(Main.LIFT_SCORING_M);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void scoringS() {
        motor.setTargetPosition(Main.LIFT_SCORING_S);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void scoringG() {
        motor.setTargetPosition(Main.LIFT_SCORING_GROUND);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

}

