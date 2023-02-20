package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class Camera {

    OpenCvCamera camera;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    public static double TagSize = 0.166;

    //Tag IDs, from the 36h11 family
    public static int AprilTag1 = 219;
    public static int AprilTag2 = 220;
    public static int AprilTag3 = 221;

    private int DetectedTag = 2;

    public Camera(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

//        telemetry.setMsTransmissionInterval(50)
    }

    public final void scanForTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;
            AprilTagDetection detectedTag = null;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == AprilTag1) {
                    this.DetectedTag = 1;
                    tagFound = true;
                    break;
                } else if (tag.id == AprilTag2) {
                    this.DetectedTag = 2;
                    tagFound = true;
                    break;
                } else if (tag.id == AprilTag3) {
                    this.DetectedTag = 3;
                    tagFound = true;
                    break;
                }
            }

        }
    }

    public final int getDetectedTag() {
        return this.DetectedTag;
    }
}
