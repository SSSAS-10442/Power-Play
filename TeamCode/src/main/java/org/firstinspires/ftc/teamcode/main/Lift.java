package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    public DcMotorEx motor;
    static final int allowance = 1;
    public int mod = 0;

    public Lift(DcMotorEx lift) {
        this.motor = lift;
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

    void stop() {
        motor.setVelocity(0);
    }

    void checkShouldStop() {
        if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < allowance) {
            stop();
        }
    }

    boolean isValidMod(int position) {
        return position + mod > 0;
    }

    void scoringL() {
        this.runToPosition(Main.LIFT_SCORING_L + mod);
    }

    void scoringM() {
        this.runToPosition(Main.LIFT_SCORING_M + mod);
    }

    void scoringS() {
        this.runToPosition(Main.LIFT_SCORING_S + mod);
    }

    void scoringG() {
        this.runToPosition(Main.LIFT_SCORING_GROUND + mod);
    }

    public void runToPosition(int position) {
        if (!isValidMod(position + mod)) {
            motor.setTargetPosition(0);
        }
        motor.setTargetPosition(position + mod);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        go();
    }

}

