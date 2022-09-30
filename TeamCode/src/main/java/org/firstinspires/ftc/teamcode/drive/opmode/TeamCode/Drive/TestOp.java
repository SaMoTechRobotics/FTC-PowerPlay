package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@TeleOp(name = "TestOp", group = "Drive")
public class TestOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        
        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("ROBOT IS WORKING?", "Yes it is!");
            telemetry.addData("Saved Drive Speed Mid", DriveSpeeds.MidDriveSpeed);
            telemetry.update();
            
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("working", "yes!");
            dashboard.sendTelemetryPacket(packet);
            

            DashboardPreview.updateField();
        }
    }
}

