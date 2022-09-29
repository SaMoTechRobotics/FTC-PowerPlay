package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TestOp", group = "Drive")
public class TestOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        

        while (opModeIsActive()) {
            telemetry.addData("ROBOT IS WORKING?", "Yes it is!");
            telemetry.update();
        }
    }
}
