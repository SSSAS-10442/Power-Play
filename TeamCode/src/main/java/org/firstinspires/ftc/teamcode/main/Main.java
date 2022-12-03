package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

enum Version {
    UNSELECTED,
    LEFT,
    RIGHT
    ;
}

public abstract class Main extends OpMode {

    public static Version version = Version.UNSELECTED;

    static int ARM_SCORING_L = 1200;
    static int ARM_SCORING_M = 0;
    static int ARM_SCORING_S = 0;
    static int ARM_SCORING_GROUND = 0;

    static int LIFT_SCORING_L = 3550;
    static int LIFT_SCORING_M = 3300;
    static int LIFT_SCORING_S = 2000;
    static int LIFT_SCORING_GROUND = 0;

}
