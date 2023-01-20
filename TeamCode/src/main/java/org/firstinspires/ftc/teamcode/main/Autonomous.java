package org.firstinspires.ftc.teamcode.main;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.NoSuchElementException;
import java.util.Vector;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Power-Play-Autonomous",group="Power-Play",preselectTeleOp="Power-Play")
public class Autonomous extends Main {

    enum State {
        STARTING_MOVEMENT,
        SET_FRAME_QUEUE_CAPACITY,
        WAIT_FOR_FRAME,
        DETECT_SIGNAL,
        SET_DETECTION,
//        FOLLOW_TRAJECTORIES,
        MOVEMENT_LR,
        MOVEMENT_BACK,
        STOP,
        ;
    }

    State state;

    ElapsedTime runtime;
    ElapsedTime statetime;

    SampleMecanumDrive smd = null;

    boolean firstFrame = true;

    int detected = 0; // 0 = not detected yet, 1 = position 1, 2 = position 2, 3 = position 3
    String text = "";

    private final MotorsEx motors = new MotorsEx();
    private Lift lift = null;
    private Claw claw = null;

    IMU imu = new IMU(this);

    public static final String VUFORIA_KEY = "AUSBY5z/////AAABmUOC52AJv0GOqfnlyjbd7h+LxEPkJlirbhJF8t0uZqXeuSkDRSypxxpINhoFZfuGIYVYJ44HHOx6rZnFa0V7PAd180mC9LDIOAR+PyVJ8T8DcimfzUe0iMAg/Ihek+5YjEW9B7XdX5Eknq3ixlhNR6lcUZfoAp5eEZ/Zjvw+OmmHI8h0yswNVdKKuqp5M5zPRlRB8Fo+IsMsvlMbK/WzHTMIFkuxceZyUQ/RBm2D4VpZXNRys6y9E8iblfPtdd7EjrJB6cpO5sDLQS5iYCqp5G9KkF3pimQrQ+Ge+SyuW1aZ0vGJvAcm3M2p1Z1gpUtVN5W1wLWPp4woLaN/q48CWKEddDMDrKDqkMRjhrJlVi52";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    WebcamName webcam;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    StandardTrackingWheelLocalizer localizerRR;
    Pose2d poseRR;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    Trajectory starting_movement;
    Trajectory move_pos_1;
    Trajectory move_pos_2;
    Trajectory move_pos_3;

    @Override
    public void init() {
        motors.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        motors.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        motors.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        motors.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motors.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motors.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor: motors.all()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        lift = new Lift(hardwareMap.get(DcMotorEx.class, "lift"));
        claw = new Claw(hardwareMap.get(Servo.class, "claw"));

        claw.close();

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        smd = new SampleMecanumDrive(hardwareMap);

        imu.initIMU();

        try {
            initVuforia();
            initTfod();
        } catch (VuforiaException ve) {
            telemetry.addLine("Failed to initialize camera!");
            telemetry.update();
        }

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
            tfod.setClippingMargins(0,0,0,0);
        } else {
            telemetry.addLine("Tfod is null!");
            telemetry.update();
        }

        starting_movement = smd.trajectoryBuilder(new Pose2d(0, 0))
                .back(-6)
                .build();

        move_pos_1 = smd.trajectoryBuilder(starting_movement.end())
                .splineTo(new Vector2d(0, -18), Math.toRadians(0))
                .splineTo(new Vector2d(12, -18), Math.toRadians(0))
                .build();

        move_pos_2 = smd.trajectoryBuilder(starting_movement.end())
                .splineTo(new Vector2d(0, -18), Math.toRadians(0))
                .build();

        move_pos_3 = smd.trajectoryBuilder(starting_movement.end())
                .splineTo(new Vector2d(0, -18), Math.toRadians(0))
                .splineTo(new Vector2d(-12, -18), Math.toRadians(0))
                .build();

        runtime = new ElapsedTime();
        statetime = new ElapsedTime();

    }

    @Override
    public void init_loop() {

        telemetry.update();

    }

    @Override
    public void start() {

        state = State.SET_FRAME_QUEUE_CAPACITY;

        runtime.reset();

    }

    @Override
    public void loop() {

        switch (state) {
            case STARTING_MOVEMENT:

                if (statetime.milliseconds() <= 500) {
                    smd.setWeightedDrivePower(new Pose2d(-0.5, 0, 0));
                    smd.update();
                } else {
                    smd.setWeightedDrivePower(new Pose2d( 0, 0, 0));
                    smd.update();
                    state = State.SET_FRAME_QUEUE_CAPACITY;
                    statetime.reset();
                    break;
                }

                break;
            case SET_FRAME_QUEUE_CAPACITY:

                if (statetime.seconds() >= 1) {
                    vuforia.setFrameQueueCapacity(1);
                    state = State.WAIT_FOR_FRAME;
                    statetime.reset();
                    break;
                }

                break;
            case WAIT_FOR_FRAME:

                if (statetime.seconds() >= 3) {
                    state = State.DETECT_SIGNAL;
                    statetime.reset();
                    break;
                }

                break;
            case DETECT_SIGNAL:
                if (statetime.seconds() > 10) {
                    text = "unknown";
                    state = State.SET_DETECTION;
                    break;
                }
                // get bitmap from camera
                Bitmap bitmap;
                try {
                    bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().element());
                } catch (NoSuchElementException e) {
                    break;
                }

                // convert bitmap into a BinaryBitmap
                int[] intArray = new int[bitmap.getWidth()*bitmap.getHeight()];
                bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
                LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(),intArray);
                BinaryBitmap bb = new BinaryBitmap(new HybridBinarizer(source));

                // create reader
                MultiFormatReader reader = new MultiFormatReader();

                // read image and get text
                Result result = null;
                try {
                    result = reader.decode(bb);
                } catch (NotFoundException e) {
                    e.printStackTrace();
                }
                try {
                    text = result.getText();
                } catch (NullPointerException e) {
                    break;
                }
                state = State.SET_DETECTION;
                statetime.reset();
                break;
            case SET_DETECTION:
                switch (text) {
                    case "https://youtube.com/watch?v=dQw4w9WgXcQ":
                        detected = 1;
                        break;
                    case "https://youtu.be/dQw4w9WgXcQ":
                        detected = 2;
                        break;
                    case "https://youtu.be/dQw4w9WgXcQ?t=0":
                        detected = 3;
                        break;
                    default:
                        detected = 4;
                        break;
                }

                state = State.MOVEMENT_LR;
                statetime.reset();
                break;
//            case FOLLOW_TRAJECTORIES:
//                if (detected == 1) {
//                    smd.followTrajectory(move_pos_1);
//                } else if (detected == 2) {
//                    smd.followTrajectory(move_pos_2);
//                } else if (detected == 3) {
//                    smd.followTrajectory(move_pos_3);
//                } else {
//                    smd.followTrajectory(move_pos_1);
//                }
//
//                state = State.STOP;
//                statetime.reset();
//                break;
            case MOVEMENT_LR:
                if (statetime.milliseconds() <= 750) {
                    if (detected == 1) {
                        smd.setWeightedDrivePower(new Pose2d(0, -1, 0));
                    } else if (detected == 2) {
                        smd.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    } else if (detected == 3) {
                        smd.setWeightedDrivePower(new Pose2d(0, 1, 0));
                    } else {
                        smd.setWeightedDrivePower(new Pose2d(0, -1, 0)); // pos 1
                    }
                    smd.update();
                } else {
                    smd.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    smd.update();
                    state = State.MOVEMENT_BACK;
                    statetime.reset();
                    break;
                }
                break;
            case MOVEMENT_BACK:
                if (statetime.milliseconds() <= 750) {
                    smd.setWeightedDrivePower(new Pose2d(-1, 0, 0));
                    smd.update();
                } else {
                    smd.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    smd.update();
                    state = State.STOP;
                    break;
                }
                break;
            case STOP:
                smd.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );

                smd.update();
                break;
            default:
                state = State.STOP;
                break;
        }

        telemetry.addLine("State: " + state);
        telemetry.addLine("Runtime: " + runtime);
        telemetry.addLine("Statetime: " + statetime);
        telemetry.addLine("Detected: " + detected);
        telemetry.addLine("Text: " + text);

        telemetry.update();

    }

    public void stop() {

    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.2f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    void initRoadRunner() {
        // This is assuming you're using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
        localizerRR = new StandardTrackingWheelLocalizer(hardwareMap);

        // Set your initial pose to x: 10, y: 10, facing 90 degrees
        localizerRR.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));
    }

    void updateRoadRunnerLocalizer() {
        // Make sure to call myLocalizer.update() on *every* loop
        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
        localizerRR.update();

        // Retrieve your pose
        poseRR = localizerRR.getPoseEstimate();
    }

}
