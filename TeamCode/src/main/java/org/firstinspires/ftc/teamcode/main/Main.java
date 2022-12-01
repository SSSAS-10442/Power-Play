package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

enum Version {
    UNSELECTED,
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
    ;
}

public abstract class Main extends OpMode {

    public static Version version = Version.UNSELECTED;

    static int ARM_SCORING_L = 200;
    static int ARM_SCORING_M = 150;
    static int ARM_SCORING_S = 100;
    static int ARM_SCORING_GROUND = 50;

    static int LIFT_SCORING_L = 2500;
    static int LIFT_SCORING_M = 2500;
    static int LIFT_SCORING_S = 1500;
    static int LIFT_SCORING_GROUND = 0;

}
