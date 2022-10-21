package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Used to hold 4 {@link DcMotorEx} objects, and run code on all of them at once.
 * */
public class MotorsEx {

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    MotorsEx() {

    }

    MotorsEx(DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    DcMotorEx[] all() {
        return new DcMotorEx[] {this.leftFront, this.leftBack, this.rightFront, this.rightBack};
    }

    void setRunMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor: all()) {
            motor.setMode(runMode);
        }
    }

}
