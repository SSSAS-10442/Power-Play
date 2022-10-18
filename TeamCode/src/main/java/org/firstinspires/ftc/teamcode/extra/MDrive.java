package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MDrive extends OpMode {

    enum State {
        MAIN,
        MOTOR,
        ;
    }

    State state;

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

    private Servo claw = null;
    private double clawPos = 0.5;
    private boolean aLastFrame = false;
    private boolean bLastFrame = false;

    @Override
    public void init() {

        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        lb = hardwareMap.get(DcMotorEx.class, "leftBack");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

        state = State.MAIN;

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        telemetry.addLine("State: " + state);

        switch (state) {
            case MAIN:
                drive = gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;

                leftFrontPower = drive + rotate + strafe;
                leftBackPower = drive + rotate - strafe;
                rightFrontPower = drive - rotate - strafe;
                rightBackPower = drive - rotate + strafe;

                lf.setVelocity(leftFrontPower);
                lb.setVelocity(leftBackPower);
                rf.setVelocity(rightFrontPower);
                rb.setVelocity(rightBackPower);

                if (gamepad1.a && !aLastFrame) {
                    clawPos += 0.05;
                }
                if (gamepad1.b && !bLastFrame) {
                    clawPos -= 0.05;
                }

                claw.setPosition(clawPos);

                aLastFrame = gamepad1.a;
                bLastFrame = gamepad1.b;

                if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                    state = State.MOTOR;
                }

                break;
            case MOTOR:

                if (gamepad1.dpad_down) {
                    lf.setVelocity(2000);
                } if (gamepad1.dpad_left) {
                    lb.setVelocity(2000);
                } if (gamepad1.dpad_up) {
                    rf.setVelocity(2000);
                } if (gamepad1.dpad_right) {
                    rb.setVelocity(2000);
                }

                if (gamepad1.start) {
                    state = State.MAIN;
                }

                telemetry.addLine("Down/LF: " + gamepad1.dpad_down);
                telemetry.addLine("Left/LB: " + gamepad1.dpad_left);
                telemetry.addLine("Up/RF: " + gamepad1.dpad_up);
                telemetry.addLine("Right/RB: " + gamepad1.dpad_right);

                break;
        }

        telemetry.update();

    }

    @Override
    public void stop() {

    }
}
