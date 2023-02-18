package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Fun;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;

@Config
@Autonomous(name = "TrajTest", group = "Fun")
public class TrajTest extends LinearOpMode {

    public static double AccelSpeed = 80;
    public static double Speed = 100;
    public static double TurnSpeed = 90;

    public static double deliverConeTime = 1;
    public static double pickupConeTime = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        Pose2d startPose = new Pose2d(40.5, -64, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime();

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(new Pose2d(40.5, -64, Math.toRadians(270)))
                        .setReversed(true) //reverse splines
                        .splineToLinearHeading(new Pose2d(35, -60, Math.toRadians(270)), Math.toRadians(90)) //clear the wall
                        .splineTo(new Vector2d(35, -46), Math.toRadians(90)) //drive around ground junction
//                                        .splineTo(new Vector2d(36, -10), Math.toRadians(90)) //drive to first high pole
                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to first high pole and turn
                        .setReversed(false) //undo reverse
//                                        .turn(Math.toRadians(90)) //turn to face stack

//                                        .lineToLinearHeading(new Pose2d(28, -10, Math.toRadians(0))) //back up to high pole
                        .waitSeconds(deliverConeTime) //deliver cone
                        .splineTo(new Vector2d(44, -12.7), Math.toRadians(-11)) //drive away from high pole
                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack

                        .waitSeconds(pickupConeTime) //pickup cone
                        .setReversed(true)
                        .splineTo(new Vector2d(59, -12), Math.toRadians(182)) //drive around ground junction
                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                        .setReversed(false)
                        .waitSeconds(deliverConeTime) //deliver cone
                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack

                        .waitSeconds(pickupConeTime) //pickup cone
                        .setReversed(true)
                        .splineTo(new Vector2d(59, -12), Math.toRadians(182)) //drive around ground junction
                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                        .setReversed(false)
                        .waitSeconds(deliverConeTime) //deliver cone
                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack

                        .waitSeconds(pickupConeTime) //pickup cone
                        .setReversed(true)
                        .splineTo(new Vector2d(59, -12), Math.toRadians(182)) //drive around ground junction
                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                        .setReversed(false)
                        .waitSeconds(deliverConeTime) //deliver cone
                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack


                        .waitSeconds(pickupConeTime) //pickup cone
                        .setReversed(true)
                        .splineTo(new Vector2d(59, -12), Math.toRadians(182)) //drive around ground junction
                        .splineTo(new Vector2d(28, -10), Math.toRadians(180)) //drive to stack
                        .setReversed(false)
                        .waitSeconds(deliverConeTime) //deliver cone

                        .splineTo(new Vector2d(59, -12), Math.toRadians(2)) //drive around ground junction
                        .splineTo(new Vector2d(64, -12), Math.toRadians(0)) //drive to stack
                        .build()
        );

        telemetry.addData("Time Remaining", 30 - timer.seconds());
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }


    }
}
