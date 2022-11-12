package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "AutoRightPro", group = "AutoPro")
public class AutoRightPro extends LinearOpMode {

  public static double strafe1 = 56.0;
  public static double back2 = 8.0;

  @Override
  public void runOpMode() throws InterruptedException {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

    Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));

    Claw Claw = new Claw(
      hardwareMap.get(Servo.class, "claw"),
      hardwareMap.get(DistanceSensor.class, "clawDistanceSensor")
    );
    Claw.close();

    Pose2d startPose = StartPose.RightPose;

    drive.setPoseEstimate(startPose);

    waitForStart();

    if (isStopRequested()) return;

    Trajectory strafeLeftTraj = drive
      .trajectoryBuilder(startPose)
      .strafeLeft(strafe1)
      .build();

    drive.followTrajectory(strafeLeftTraj);

    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

    sleep(2000);

    Trajectory backTraj = drive
      .trajectoryBuilder(strafeLeftTraj.end())
      .back(back2)
      .build();

    drive.followTrajectory(backTraj);

    Arm.setRotation(ArmRotation.Left);

    sleep(1000);

    Slide.setHeight(SlideHeight.HighPole - 5, SlideSpeed.Mid);

    sleep(2000);

    Claw.open();

    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid);

    sleep(2000);

    Arm.setRotation(ArmRotation.Center);
    Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max);
    Claw.close();

    sleep(10000);
  }
}