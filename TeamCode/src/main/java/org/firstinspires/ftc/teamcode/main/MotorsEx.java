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

    void setVelocity(double lf, double rf, double lb, double rb) {
        leftFront.setVelocity(lf);
        rightFront.setVelocity(rf);
        leftBack.setVelocity(lb);
        rightBack.setVelocity(rb);
    }

}
