package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    public DcMotorEx motor;

    public Arm(DcMotorEx arm) {
        this.motor = arm;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void go() {
        go(3000);
    }

    void go(int velocity) {
        motor.setVelocity(velocity);
    }

    void scoringL() {
        this.runToPosition(Main.ARM_SCORING_L);
    }

    void scoringM() {
        this.runToPosition(Main.ARM_SCORING_M);
    }

    void scoringS() {
        this.runToPosition(Main.ARM_SCORING_S);
    }

    void scoringG() {
        this.runToPosition(Main.ARM_SCORING_GROUND);
    }

    void runToPosition(int position) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

}
