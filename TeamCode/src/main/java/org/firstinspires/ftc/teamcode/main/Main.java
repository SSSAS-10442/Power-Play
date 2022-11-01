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

    static int ARM_SCORING_L = 1400;
    static int ARM_SCORING_M = 900;
    static int ARM_SCORING_S = 400;
    static int ARM_SCORING_GROUND = 100;

    static int LIFT_SCORING_L = 1400;
    static int LIFT_SCORING_M = 900;
    static int LIFT_SCORING_S = 400;
    static int LIFT_SCORING_GROUND = 100;

}
