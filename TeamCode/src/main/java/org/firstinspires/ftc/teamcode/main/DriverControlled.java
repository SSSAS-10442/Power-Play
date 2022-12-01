package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="Power-Play",group="Power-Play")
public class DriverControlled extends Main {

    enum State {
        HOME,
        SCORING_S,
        SCORING_M,
        SCORING_L,
        ;
    }

    // Create and initialize a new State object
    private State state;

    LastKey lastkey;

    // Create and initialize a new MotorsEx object (see MotorsEx.java)
    private final MotorsEx motors = new MotorsEx();

    // Create the claw object
    private Claw claw = null;

    // Create the Arm object
    private Arm arm = null;

    // Create the Lift object
    private Lift lift = null;

    // Create and initialize drive, strafe, and rotate. These control driving with mechanum wheels.
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;

    // Create and initialize these variables. These control the velocity of each motor.
    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private void driving() {
        driving(2000);
    }

    /**
     * Calls driving() with parameter max set to 0
     * */
    private void noDriving() {
        driving(0);
    }

    /**
     * Updates the drive, strafe, and rotate variables to be used in the calculations for the power variables.
     * drive, strafe, and rotate are then multiplied by max
     * */
    private void driving(int max) {
        drive = -gamepad1.left_stick_y * max;
        strafe = gamepad1.left_stick_x * max;
        rotate = gamepad1.right_stick_x * max / 2;
    }

    /**
     * Update telemetry with useful information like the state and inputs.
     * */
    private void updateTelemetry() {
        telemetry.addData("State", state);
        telemetry.addData("Version", version);
        telemetry.addData("Lift Mod", lift.mod);
//        telemetry.addLine("----------");
//        telemetry.addData("(x, y)", "(" + poseRR.getX() + ", " + poseRR.getY() + ")");
//        telemetry.addData("heading", poseRR.getHeading());
        telemetry.update();
    }

    private void updateLiftMod() {
        if (gamepad1.y) {
            lift.mod += 5;
        } if (gamepad1.x) {
            lift.mod -= 5;
        }
    }

    private void checkArmMovement() {
        if (gamepad1.b) {
            arm.motor.setPower(1);
        } else if (gamepad1.a) {
            arm.motor.setPower(-0.8);
        } else {
            arm.motor.setPower(0);
        }
    }

    private void checkClawKeybinds() {
        if (gamepad1.x) {
            claw.close();
        } else if (gamepad1.y) {
            claw.open();
        }
    }

    /**
     * Initialize motors and other devices
     * */
    @Override
    public void init() {
        // Initialize motors
        motors.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        motors.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        motors.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set motor directions
        motors.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motors.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motors.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run using encoder
        motors.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize claw
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));

        // Initialize arm
        arm = new Arm(hardwareMap.get(DcMotorEx.class, "arm"));
        arm.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // REMOVE THIS ONCE WE GET AN ENCODER ON ARM

        // Initialize lift
        lift = new Lift(hardwareMap.get(DcMotorEx.class,"lift"));

        // Start recording last inputs
        lastkey = new LastKey(gamepad1);

        // Initialize RoadRunner
//        initRoadRunner();

        // Set state to HOME
        state = State.HOME;
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    /**
     * This is the driver controlled period. Our state machine is here.
     * */
    @Override
    public void loop() {

        // Begin our state machine
        switch (state) {
            case HOME:
                driving(); // Updates driving using gamepad1 inputs
                checkClawKeybinds();

                // Bring arm and lift down
//                arm.runToPosition(0);
                lift.runToPosition(0);

                updateLiftMod();

                // This is temporary. We only need this until we have an encoder on the arm motor
                checkArmMovement();

                // If dpad_up is pressed, switch to GRAB_CONE state
                if (gamepad1.dpad_left) {
                    state = State.SCORING_S;
                } if (gamepad1.dpad_up) {
                    state = State.SCORING_M;
                } if (gamepad1.dpad_right) {
                    state = State.SCORING_L;
                } if (gamepad1.dpad_down) {
                    state = State.HOME;
                }
                break;
            case SCORING_L:
                driving();
                checkClawKeybinds();

                // Bring arm and lift to SCORING_L position
//                arm.scoringL();
                lift.scoringL();

                updateLiftMod();

                // This is temporary. We only need this until we have an encoder on the arm motor
                checkArmMovement();

                // If dpad_down is held, open claw
                if (gamepad1.dpad_down) {
                    claw.open();
                }

                if (gamepad1.dpad_left) {
                    state = State.SCORING_S;
                } if (gamepad1.dpad_up) {
                    state = State.SCORING_M;
                } if (gamepad1.dpad_right) {
                    state = State.SCORING_L;
                } if (gamepad1.dpad_down) {
                    state = State.HOME;
                }
                break;
            case SCORING_M:
                driving();
                checkClawKeybinds();

                // Bring arm and lift to SCORING_M position
//                arm.scoringM();
                lift.scoringM();

                updateLiftMod();

                // This is temporary. We only need this until we have an encoder on the arm motor
                checkArmMovement();

                if (gamepad1.dpad_down) {
                    claw.open();
                }

                if (gamepad1.dpad_left) {
                    state = State.SCORING_S;
                } if (gamepad1.dpad_up) {
                    state = State.SCORING_M;
                } if (gamepad1.dpad_right) {
                    state = State.SCORING_L;
                } if (gamepad1.dpad_down) {
                    state = State.HOME;
                }
                break;
            case SCORING_S:
                driving();
                checkClawKeybinds();

                // Bring arm and lift to SCORING_S position
//                arm.scoringS();
                lift.scoringS();

                updateLiftMod();

                // This is temporary. We only need this until we have an encoder on the arm motor
                checkArmMovement();

                if (gamepad1.dpad_down) {
                    claw.open();
                }

                if (gamepad1.dpad_left) {
                    state = State.SCORING_S;
                } if (gamepad1.dpad_up) {
                    state = State.SCORING_M;
                } if (gamepad1.dpad_right) {
                    state = State.SCORING_L;
                } if (gamepad1.dpad_down) {
                    state = State.HOME;
                }
                break;
            default:
                // If somehow we get to the default, go to HOME state
                state = State.HOME;
                break;
        }

        // Update telemetry
        updateTelemetry();
        lastkey.update();

        // Calculate power for each wheel
        leftFrontPower = drive + rotate + strafe;
        leftBackPower = drive + rotate - strafe;
        rightFrontPower = drive - rotate - strafe;
        rightBackPower = drive - rotate + strafe;

        // Send power to wheels
        motors.leftFront.setVelocity(leftFrontPower);
        motors.leftBack.setVelocity(leftBackPower);
        motors.rightFront.setVelocity(rightFrontPower);
        motors.rightBack.setVelocity(rightBackPower);

    }

    @Override
    public void stop() {

    }

}
