package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "AutoForward", group = "Auto")
public class AutoForward extends LinearOpMode {

  public static double forward1 = 42.0;

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Pose2d startPose = StartPose.RightPose;

    drive.setPoseEstimate(startPose);

    waitForStart();

    if (isStopRequested()) return;

    Trajectory forwardTraj = drive
      .trajectoryBuilder(startPose)
      .forward(forward1)
      .build();

    drive.followTrajectory(forwardTraj);
  }
}
