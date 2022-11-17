package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Config
public class Camera {

    public static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {"1 Bolt", "2 Bulb", "3 Panel"};

    private static final String VUFORIA_KEY =
            "Abp0NU7/////AAABmQedb7TAVUQatPEi5Nz7wMISMvR04YthYTgVl234gSVsQt8WS71fidOvJDcokKxADcc2bb8Qy44vEOU8YMM6lwH3wYBMF3fFG49XWRHU1MYUDsx93i1rwmSYt0SOehFf3dsnN5QBHJRrb0gYHPVV9rOal3ZdvXHS8pnAgnWxueyo5Ctsn4pU3qUox9lEgM8ZpKnKJHeYGWoLV+zuerA+dr+SpQj4CCSv9qsAc9vCS+iI46cKTuzgBG8navGrn8r9LO9yKuXOVkjD3l0rxLNH54FNfKWvmxTsJR3jKGzW8fCuMdEpHESGwQeMr34KZ9tyaeB12fdnZI+XjgjIlKRv6aSWSP0BQJPK1fZIUkWdsOID";
    private static final double ZOOM_FACTOR = 2.5;
    private static final double ASPECT_RATIO = 16.0 / 9.0;
    private static final float MIN_CONFIDENCE = 0.65f;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public Camera(HardwareMap hardwareMap, FtcDashboard dashboard) {
        initVuforia();
        initTfod(hardwareMap, dashboard);

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(ZOOM_FACTOR, ASPECT_RATIO);
        }
    }

    public final void update(Telemetry telemetry) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(
                            recognition.getRight() - recognition.getLeft()
                    );
                    double height = Math.abs(
                            recognition.getTop() - recognition.getBottom()
                    );

                    telemetry.addData("", " ");
                    telemetry.addData(
                            "Image",
                            "%s (%.0f %% Conf.)",
                            recognition.getLabel(),
                            recognition.getConfidence() * 100
                    );
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData(
                            "- Size (Width/Height)",
                            "%.0f / %.0f",
                            width,
                            height
                    );
                }
                telemetry.update();
            }
        }
    }

    public final void getResults() {
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap, FtcDashboard dashboard) {
        int tfodMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier(
                        "tfodMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(
                tfodMonitorViewId
        );
        tfodParameters.minResultConfidence = MIN_CONFIDENCE; //0.75f
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod =
                ClassFactory
                        .getInstance()
                        .createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        dashboard.startCameraStream(vuforia, 0);
    }
}
