package org.firstinspires.ftc.teamcode.main;

import android.graphics.Bitmap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Power-Play-Autonomous",group="Power-Play",preselectTeleOp="Power-Play")
public class Autonomous extends Main {

    enum State {
        WAIT_FOR_ELEMENT,
        DETECT_SIGNAL,
        SET_DETECTION,
        SET_TRAJECTORIES,
        MOVEMENT,
        MOVEMENT_2,
        STOP,
        ;
    }

    State state;

    ElapsedTime runtime;

    SampleMecanumDrive smd = null;
    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;

    boolean firstFrame = true;

    int detected = 0; // 0 = not detected yet, 1 = position 1, 2 = position 2, 3 = position 3
    String text = "";

    private final MotorsEx motors = new MotorsEx();
    private Lift lift = null;
    private Claw claw = null;

    IMU imu = new IMU(this);

    public static final String VUFORIA_KEY = "AbKdfgn/////AAABmUwUEpbO9EDrlhxJl1yQGb87fcS3l27IKj8Y4mqrbhrHnM4bscBBBLlRPyUn/9cGTLFxs76UUgqnZ/nQenLu++B4lsjlLdq3f8M3oI+fbqITBOnMCYbNFRtsVOdgpJzVkrRarkFnMKN7UPpMKs4xI0t0ufb+hz+0H7d1GnkMtYRHG/n1NmSUCN5uW6seDdyIi0yObCByy/uM8grVrJQbTNEiAjlBZ6ahKtgWoGSmnhjkD4tCgtLTHRMUKoCOd6yAoFdrLI6tVL2JUAGuaF9X81ryvKPk5m4bNXHDxjSqv3gpeuQKOhBnnmgnwwYuy2H65aspPk6mO/nRCp6esV75TK2ASBtn0AI5I0XEIUSxIb5P";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    CameraName webcam;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    StandardTrackingWheelLocalizer localizerRR;
    Pose2d poseRR;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

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

        webcam = hardwareMap.get(CameraName.class, "Webcam 1");

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
            tfod.setClippingMargins(0,0,0,0); // This will need to be updated
        }

        vuforia.setFrameQueueCapacity(1);

        runtime = new ElapsedTime();

    }

    @Override
    public void init_loop() {

        telemetry.addLine("SELECT A VERSION RIGHT NOW!!! A = Left, B = Right");
        telemetry.addLine("Version: " + version);

        if (gamepad1.a) {
            version = Version.LEFT;
        } if (gamepad1.b) {
            version = Version.RIGHT;
        }

        telemetry.update();

    }

    @Override
    public void start() {

        state = State.DETECT_SIGNAL;

        runtime.reset();

    }

    @Override
    public void loop() {

        switch (state) {
            case WAIT_FOR_ELEMENT:
                if (runtime.seconds() > 1) {
                    state = State.DETECT_SIGNAL;
                    firstFrame = true;
                    break;
                }
                firstFrame = false;
                break;
            case DETECT_SIGNAL:
                if (runtime.seconds() > 10) {
                    text = "unknown";
                    state = State.SET_DETECTION;
                    firstFrame = true;
                    break;
                }
                // get bitmap from camera
                Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().element());

                // convert bitmap into a BinaryBitmap
                int[] intArray = new int[bitmap.getWidth()*bitmap.getHeight()];
                bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
                LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(),intArray);
                BinaryBitmap bb = new BinaryBitmap(new HybridBinarizer(source));

                // create reader
                MultiFormatReader reader = new MultiFormatReader();

                firstFrame = false;

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
                firstFrame = true;
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
                state = State.SET_TRAJECTORIES;
                firstFrame = true;
                break;
            case SET_TRAJECTORIES:
                traj1 = smd.trajectoryBuilder(new Pose2d())
                        .forward(10)
                        .build();

                if (version.equals(Version.RIGHT)) {
                    traj2 = smd.trajectoryBuilder(traj1.end())
                            .strafeLeft(5)
                            .build();
                } else {
                    traj2 = smd.trajectoryBuilder(traj1.end())
                            .strafeRight(5)
                            .build();
                }

                traj3 = smd.trajectoryBuilder(traj2.end())
                        .forward(2)
                        .build();

                traj4 = smd.trajectoryBuilder(traj3.end())
                        .back(2)
                        .build();

                if (detected == 1) {
                    traj5 = smd.trajectoryBuilder(traj4.end())
                            .strafeLeft(10)
                            .build();
                } else if (detected == 2 || detected == 4) {
                    traj5 = null;
                } else {
                    traj5 = smd.trajectoryBuilder(traj4.end())
                            .strafeRight(10)
                            .build();
                }
                state = State.MOVEMENT;
                break;
            case MOVEMENT:
                if (firstFrame) {
                    claw.close();
                    smd.followTrajectory(traj1); // forward
                    smd.followTrajectory(traj2); // sideways
                    smd.followTrajectory(traj3); // forward (slight)
                    lift.scoringM();
                }
                if (Math.abs(lift.motor.getTargetPosition() - lift.motor.getCurrentPosition()) < 5) {
                    state = State.MOVEMENT_2;
                    firstFrame = true;
                    break;
                }
                firstFrame = false;
                break;
            case MOVEMENT_2:
                claw.open();
                lift.runToPosition(0);
                smd.followTrajectory(traj4); // back (slight)
                if (traj5 != null) {
                    smd.followTrajectory(traj5); // sideways
                }

                state = State.STOP;
                break;
            case STOP:
                break;
            default:
                state = State.STOP;
                break;
        }

        telemetry.addLine("Version: " + version);
        telemetry.addLine("State: " + state);
        telemetry.addLine("Runtime: " + runtime);
        telemetry.addLine("Detected: " + detected);
        telemetry.addLine("Text: " + text);

        telemetry.update();

    }

    public void stop() {

    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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
