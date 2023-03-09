package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Drive.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;

//@TeleOp(name = "ColorTest", group = "Tests")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        waitForStart();

        while (opModeIsActive()) {

            SensorColors.Color detectedColor = SensorColors.detectColor(colorSensor);

            int ParkingPosition = SensorColors.getParkingPosition(detectedColor);


            telemetry.addData("Detected Color", detectedColor);
            telemetry.addLine("");
            telemetry.addData("Detected Parking Position", ParkingPosition);
            telemetry.addLine("");
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();

        }

    }
}
