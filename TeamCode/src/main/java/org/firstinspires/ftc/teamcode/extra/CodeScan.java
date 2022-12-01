package org.firstinspires.ftc.teamcode.extra;

import android.graphics.Bitmap;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;
import org.firstinspires.ftc.teamcode.main.LastKey;

@Disabled
@TeleOp
public class CodeScan extends OpMode {

    LastKey last = new LastKey(gamepad2);

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
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

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
    }

    @Override
    public void loop() {
        if (gamepad2.a && !last.a) {
            Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().element());

            int[] intArray = new int[bitmap.getWidth()*bitmap.getHeight()];
            //copy pixel data from the Bitmap into the 'intArray' array
            bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());

            LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(),intArray);

            BinaryBitmap bb = new BinaryBitmap(new HybridBinarizer(source)); // HybridBinarizer(source).getBlackMatrix() returns a BitMatrix
            Reader reader = new MultiFormatReader();

            Result result = null;
            try {
                result = reader.decode(bb);
            } catch (NotFoundException | ChecksumException | FormatException e) {
                e.printStackTrace();
            }
            telemetry.addLine("QR Result: " + result.getText());
            telemetry.update();
        }
        last.update();
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
