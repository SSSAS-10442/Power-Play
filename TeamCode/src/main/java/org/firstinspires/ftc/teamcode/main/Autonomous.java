package org.firstinspires.ftc.teamcode.main;

import android.graphics.Bitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.datamatrix.DataMatrixReader;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Power-Play-Autonomous",group="Power-Play",preselectTeleOp="Power-Play")
public class Autonomous extends Main {

    enum State {
        SET_VUFORIA_FRAME_QUEUE,
        EXAMPLE_A,
        EXAMPLE_B,
        DETECT_SIGNAL,
        EXAMPLE_C,
        ;
    }

    State state;

    ElapsedTime runtime;

    int detected = 0; // 0 = not detected yet, 1 = position 1, 2 = position 2, 3 = position 3

    private final MotorsEx motors = new MotorsEx();

    IMU imu = new IMU(this);

    public static final String VUFORIA_KEY = "AbKdfgn/////AAABmUwUEpbO9EDrlhxJl1yQGb87fcS3l27IKj8Y4mqrbhrHnM4bscBBBLlRPyUn/9cGTLFxs76UUgqnZ/nQenLu++B4lsjlLdq3f8M3oI+fbqITBOnMCYbNFRtsVOdgpJzVkrRarkFnMKN7UPpMKs4xI0t0ufb+hz+0H7d1GnkMtYRHG/n1NmSUCN5uW6seDdyIi0yObCByy/uM8grVrJQbTNEiAjlBZ6ahKtgWoGSmnhjkD4tCgtLTHRMUKoCOd6yAoFdrLI6tVL2JUAGuaF9X81ryvKPk5m4bNXHDxjSqv3gpeuQKOhBnnmgnwwYuy2H65aspPk6mO/nRCp6esV75TK2ASBtn0AI5I0XEIUSxIb5P";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    CameraName webcam;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

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

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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

        runtime = new ElapsedTime();

    }

    @Override
    public void init_loop() {

        telemetry.addLine("Version: " + version);

        if (gamepad1.a) {
            version = Version.RED_LEFT;
        } if (gamepad1.b) {
            version = Version.RED_RIGHT;
        } if (gamepad1.x) {
            version = Version.BLUE_LEFT;
        } if (gamepad1.y) {
            version = Version.BLUE_RIGHT;
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

        telemetry.addLine("Version: " + version);
        telemetry.addLine("State: " + state);
        telemetry.addLine("Runtime: " + runtime);
        telemetry.addLine("Detected: " + detected);

        switch (state) {
            case SET_VUFORIA_FRAME_QUEUE:
                vuforia.setFrameQueueCapacity(1);

                if (runtime.seconds() > 3) {
                    state = State.DETECT_SIGNAL;
                }
                break;
            case EXAMPLE_A:
                return;
            case EXAMPLE_B:
                break;
            case DETECT_SIGNAL:
                Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().element());

                int[] intArray = new int[bitmap.getWidth()*bitmap.getHeight()];
                //copy pixel data from the Bitmap into the 'intArray' array
                bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
                LuminanceSource source = new com.google.zxing.RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(),intArray);

                BinaryBitmap bb = new BinaryBitmap(new HybridBinarizer(source));

                Result qrCodeResult = null;
                try {
                    qrCodeResult = new DataMatrixReader().decode(bb);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                String text = qrCodeResult.getText();
                telemetry.addLine("QR Result: " + text);
                state = State.EXAMPLE_A;
                break;
            case EXAMPLE_C:
                break;
            default:
                break;
        }

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

}
