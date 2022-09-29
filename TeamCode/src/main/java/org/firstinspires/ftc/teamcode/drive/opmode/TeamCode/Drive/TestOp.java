package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "TestOp", group = "Drive")
public class TestOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        

        while (opModeIsActive()) {
            telemetry.addData("ROBOT IS WORKING?", "Yes it is!");
            telemetry.addData("Saved Drive Speed Mid", DriveSpeeds.MidDriveSpeed);
            telemetry.update();
        }
    }
}
