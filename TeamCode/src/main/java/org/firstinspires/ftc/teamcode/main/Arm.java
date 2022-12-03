package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm {

    public DcMotorEx motor;
    static final int allowance = 1;
    public int mod = 0;

    public Arm(DcMotorEx arm) {
        this.motor = arm;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    boolean isValidMod(int position) {
        return position + mod > 0;
    }

    void go() {
        go(3000);
    }

    void go(int velocity) {
        motor.setVelocity(velocity);
    }

    void scoringL() {
        mod = 0;
        this.runToPosition(Main.ARM_SCORING_L);
    }

    void scoringM() {
        mod = 0;
        this.runToPosition(Main.ARM_SCORING_M);
    }

    void scoringS() {
        mod = 0;
        this.runToPosition(Main.ARM_SCORING_S);
    }

    void scoringG() {
        this.runToPosition(Main.ARM_SCORING_GROUND);
    }

    void runToPosition(int position) {
        motor.setTargetPosition(position + mod);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

}
