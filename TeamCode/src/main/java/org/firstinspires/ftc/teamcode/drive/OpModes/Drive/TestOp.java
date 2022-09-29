package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TestOp", group = "Drive")
public class TestOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        

        while (opModeIsActive()) {
            if (TestStorage.SecureYay == "Yay!") {
                telemetry.addData("Updated Saved Data", "");
                TestStorage.SecureYay = "Let's go!";
            }

            telemetry.addData("ROBOT IS WORKING?", "Yes it is!");
            telemetry.addData("Saved data", TestStorage.SecureYay);
            telemetry.update();
        }
    }
}
