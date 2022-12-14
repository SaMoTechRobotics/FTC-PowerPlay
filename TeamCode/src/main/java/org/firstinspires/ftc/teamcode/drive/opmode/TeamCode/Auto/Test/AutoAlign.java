//package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorDistances;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
//import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
//
//@Config
//@Autonomous(name = "AutoAlign", group = "Test")
//public class AutoAlign extends LinearOpMode {
//
//    public static double speed = 0.2;
////
////    public static double detectAmount = 14.0;
////    public static double placeDistance = 3;
////
////    public static double placeMargin = 0.25;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//
//
//        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));
//
//        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
//        Arm.setRotation(ArmRotation.Center);
//
//        Claw Claw = new Claw(
//                hardwareMap.get(Servo.class, "claw"),
//                hardwareMap.get(DistanceSensor.class, "leftDistanceSensor")
//        );
//        Claw.close();
//
//        Pose2d startPose = new Pose2d(-60, -48, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPose);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        sleep(1000);
//
//        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//        sleep(2000);
//
//
////        while (sensor.getDistance(DistanceUnit.INCH) > detectAmount && opModeIsActive()) {
////            drive.setWeightedDrivePower(
////                    new Pose2d(
////                            -speed,
////                            0,
////                            0
////                    )
////            );
////            drive.update();
////            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
////            telemetry.update();
////        }
////
////        drive.setWeightedDrivePower(
////                new Pose2d(
////                        0,
////                        0,
////                        0
////                )
////        );
//        drive.alignWithPoleAsync(sensor, SensorDistances.DetectAmount, opModeIsActive());
//
//        Arm.setRotation(ArmRotation.Left);
//
//        drive.alignPlaceDistanceAsync(sensor, SensorDistances.PlaceDistance, SensorDistances.PlaceMargin, opModeIsActive());
//
//
////        while (sensor.getDistance(DistanceUnit.INCH) > placeDistance && opModeIsActive()) {
////            drive.setWeightedDrivePower(
////                    new Pose2d(
////                            0,
////                            speed,
////                            0
////                    )
////            );
////            drive.update();
////            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
////            telemetry.update();
////        }
////
////        drive.setWeightedDrivePower(
////                new Pose2d(
////                        0,
////                        0,
////                        0
////                )
////        );
//
//        Slide.setHeight(SlideHeight.HighPole - SlideHeight.PlaceMargin, SlideSpeed.Max);
//
//        sleep(1000);
//
//        Claw.open();
//
//        sleep(1000);
//
//        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
//
//        Arm.setRotation(ArmRotation.Center);
//
//        sleep(1000);
//
//        drive.alignPlaceDistanceAsync(sensor, SensorDistances.CenterDistance, SensorDistances.PlaceMargin, opModeIsActive());
//
//
//    }
//}
