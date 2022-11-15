package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {

    public static double speed = 0.1;

    public static double detectAmount = 3.0;
    public static double placeDistance = 1.5;

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "clawDistanceSensor");

    waitForStart();

    if (isStopRequested()) return;

    
  }
}
