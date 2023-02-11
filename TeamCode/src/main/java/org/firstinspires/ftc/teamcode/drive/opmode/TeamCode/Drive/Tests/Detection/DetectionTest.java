package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests.Detection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Camera;

//@Disabled
@TeleOp
public class DetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Camera Camera = new Camera(hardwareMap);


        while (!isStarted() && !isStopRequested()) {
            Camera.scanForTags();
            telemetry.addData("Detected", "Tag " + Camera.getDetectedTag());
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Parking in Position", Camera.getDetectedTag());
            telemetry.update();
        }
    }
}