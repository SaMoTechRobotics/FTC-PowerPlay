package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;


@TeleOp(name = "SlideTest", group = "TeleOp")
public class SlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        DcMotor slide = hardwareMap.dcMotor.get("slide");

        waitForStart();

        while (opModeIsActive()) {

            slide.setPower(gamepad1.left_stick_y * 0.1);

            telemetry.addData("Slide Power", slide.getPower());
            telemetry.update();

        }
    }
}