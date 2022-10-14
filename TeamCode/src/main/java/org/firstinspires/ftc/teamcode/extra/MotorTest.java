package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {

    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    public void init() {
        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lb = hardwareMap.get(DcMotor.class, "leftBack");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");
    }

    public void loop() {

        if (gamepad1.a) {
            lf.setPower(1);
        } else if (gamepad1.b) {
            lb.setPower(1);
        } else if (gamepad1.x) {
            rf.setPower(1);
        } else if (gamepad1.y) {
            rb.setPower(1);
        }

        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        telemetry.addLine("A: LF");
        telemetry.addLine("B: LB");
        telemetry.addLine("X: RF");
        telemetry.addLine("Y: RB");
        telemetry.update();

    }

}
