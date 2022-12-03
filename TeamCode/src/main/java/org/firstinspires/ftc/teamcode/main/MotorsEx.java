package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Used to hold 4 {@link DcMotorEx} objects, and run code on all of them at once.
 * */
public class MotorsEx {

    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;

    public MotorsEx() {

    }

    public MotorsEx(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        for (DcMotorEx motor: this.all()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public DcMotorEx[] all() {
        return new DcMotorEx[] {this.leftFront, this.leftBack, this.rightFront, this.rightBack};
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor: all()) {
            motor.setMode(runMode);
        }
    }

}
