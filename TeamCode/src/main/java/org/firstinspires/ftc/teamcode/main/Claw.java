package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo servo;

    Claw(Servo claw) {
        this.servo = claw;
    }

    void close() {
        servo.setPosition(0.7);
    }

    void open() {
        servo.setPosition(0.4);
    }

}
