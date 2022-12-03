package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Lift;

@Disabled
@TeleOp(group="Power-Play-Extra")
public class RaiseLift extends OpMode {

    Lift lift = null;

    @Override
    public void init() {
        lift = new Lift(hardwareMap.get(DcMotorEx.class, "lift"));
    }

    @Override
    public void loop() {
        lift.runToPosition(2500);
    }
}
