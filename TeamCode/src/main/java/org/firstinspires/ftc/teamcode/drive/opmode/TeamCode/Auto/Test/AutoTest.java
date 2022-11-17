package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;

@Config
@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends LinearOpMode {

    public static double speed = 0.5;

    public static double detectAmount = 12.0;
    public static double placeDistance = 5.0;

    public static double correctAmount = 1.0;

    public static double moveDistance = 40.0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw = new Claw(
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "leftDistanceSensor")
        );
        Claw.close();

        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        sleep(1000);

        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        sleep(2000);


        Trajectory detectTraj =
                new TrajectoryBuilder(startPose, new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return speed;
                    }
                }, new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30.0;
                    }
                }).back(moveDistance).build();

        drive.followTrajectoryAsync(
                detectTraj
        );

        while (sensor.getDistance(DistanceUnit.INCH) > detectAmount && opModeIsActive()) {
            drive.update();
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        drive.breakFollowing();
        drive.setWeightedDrivePower(
                new Pose2d(
                        0,
                        0,
                        0
                )
        );


        Arm.setRotation(ArmRotation.Left);

        drive.followTrajectory(
                new TrajectoryBuilder(startPose, new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return speed;
                    }
                }, new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30.0;
                    }
                }).back(correctAmount).build()
        );

//        Trajectory placeTrajLeft = new TrajectoryBuilder(detectTraj.end(), new TrajectoryVelocityConstraint() {
//            @Override
//            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
//                return speed;
//            }
//        }, new TrajectoryAccelerationConstraint() {
//            @Override
//            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
//                return 30.0;
//            }
//        }).strafeLeft(moveDistance).build();
//
////        boolean side = false;
////        if (sensor.getDistance(DistanceUnit.INCH) > placeDistance) {
////            drive.followTrajectoryAsync(placeTrajLeft);
////        } else {
////            drive.followTrajectoryAsync(placeTrajRight);
////            side = true;
////        }
//
//        drive.followTrajectoryAsync(placeTrajLeft);

        Trajectory placeTraj =
                new TrajectoryBuilder(startPose, new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return speed;
                    }
                }, new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 30.0;
                    }
                }).strafeLeft(moveDistance).build();

        drive.followTrajectoryAsync(
                placeTraj
        );

        while (sensor.getDistance(DistanceUnit.INCH) > placeDistance && opModeIsActive()) {
            drive.update();
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        drive.breakFollowing();
        drive.setWeightedDrivePower(
                new Pose2d(
                        0,
                        0,
                        0
                )
        );

        sleep(2000);

        Claw.open();

        sleep(2000);

        Arm.setRotation(ArmRotation.Center);

        sleep(1000);


    }
}
