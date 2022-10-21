package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

enum Version {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
    ;
}

public abstract class Main extends OpMode {

    public static Version version = null;

    static int SCORING_L = 1400;
    static int SCORING_M = 900;
    static int SCORING_S = 400;
    static int SCORING_GROUND = 100;

}
