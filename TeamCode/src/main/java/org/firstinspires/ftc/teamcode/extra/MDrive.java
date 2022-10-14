package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.main.MotorsEx;

public class MDrive extends OpMode {

    DcMotorEx lf = null;
    DcMotorEx rf = null;
    DcMotorEx rb = null;
    DcMotorEx lb = null;

    double drive = 0;
    double strafe = 0;
    double rotate = 0;

    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;
    double leftBackPower = 0;

    @Override
    public void init() {

        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        lb = hardwareMap.get(DcMotorEx.class, "leftBack");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        drive = gamepad1.left_stick_y * 2000;
        strafe = gamepad1.left_stick_x * 2000;
        rotate = gamepad1.right_stick_x * 2000;

        leftFrontPower = rotate - strafe + drive;
        rightFrontPower = rotate - strafe - drive;
        leftBackPower = rotate + strafe + drive;
        rightBackPower = rotate + strafe - drive;

        lf.setVelocity(leftFrontPower);
        rf.setVelocity(rightFrontPower);
        rb.setVelocity(rightBackPower);
        lb.setVelocity(leftBackPower);
        
    }

    @Override
    public void stop() {

    }
}
