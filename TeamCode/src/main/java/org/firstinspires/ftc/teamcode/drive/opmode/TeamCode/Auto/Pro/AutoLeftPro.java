package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import static org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight.StackConeHeights;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Auto.AutoAlignManager;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Camera;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoPositions;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TargetPole;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.Vec2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@SuppressWarnings("ALL")
@Config
@Autonomous(name = "AutoLeftPro", group = "A")
public class AutoLeftPro extends LinearOpMode {

    /*
     * Global Vars ---------------------------------------------------------------------------------
     */
    public final static int SIDE = AutoSide.Left;
    public static final double FINAL_ROT = SIDE == AutoSide.Right ? 0 : 180;

    public static boolean ReadEveryOther = true;
    public static boolean ReadEveryTime = true;

    /*
     * Trajector Speed Vars -----------------------------------------------------------------------
     */

    private static class TrajectorySpeeds {
        public static double MaxSpeed = 90;
        public static double FastSpeed = 60;
        public static double NormalSpeed = 55;

        public static double MaxAccel = 60;
        public static double FastAccel = 45;
        public static double NormalAccel = 35;

        public static double MaxTurn = 180;
        public static double FastTurn = 100;
        public static double NormalTurn = 60;
    }

    public static TrajectorySpeeds TRAJECTORY_SPEEDS = new TrajectorySpeeds();

    /*
     * Trajectory Vars -----------------------------------------------------------------------------
     */

    private static class TrajectoryDistances {
        public static double ClearPole = 2;

        public static double ForwardPickupX = 64;

        public static double PickupOffset = 1;


        public static double StackY = -12;

    }

    public static TrajectoryDistances TRAJECTORY_DISTANCES = new TrajectoryDistances();


    /*
     * Position Vars -------------------------------------------------------------------------------
     */

    private static class TrajectoryLocations {

        public static Vec2 FirstDrive = SIDE == AutoSide.Right ? new Vec2(35, -10) : new Vec2(35, -10);

        // Stack pos                                                right                       left
        public static Vec2 StackPos = SIDE == AutoSide.Right ? new Vec2(63, TrajectoryDistances.StackY) : new Vec2(63, TrajectoryDistances.StackY);

        public static Vec2 AlignStackPos = SIDE == AutoSide.Right ? new Vec2(47, TrajectoryDistances.StackY) : new Vec2(47, TrajectoryDistances.StackY);

        public static Vec2 AvoidByStackPos = SIDE == AutoSide.Right ? new Vec2(46, TrajectoryDistances.StackY) : new Vec2(46, TrajectoryDistances.StackY);

        public static class PolePos {
            public double X;
            public double Y;
            public double GiveUpX; // If the robot is past this X value, it will give up on finding the pole
            public double FindX; // The robot will start looking for the pole at this X value

            public double Adjust; // The robot will adjust its position by this amount by driving back

            public PolePos(double findX, double x, double giveUpX, double y, double adjust) {
                X = x;
                Y = y;
                GiveUpX = giveUpX;
                FindX = findX;
                Adjust = adjust;
            }
        }

        public static PolePos FarHighPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(8, 4, 0, -13.5, 0) :
                        new PolePos(8, 4, 0, -14, 0); //left
        public static PolePos CloseHighPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(31, 28, 20, TrajectoryDistances.StackY, 0) : //right
                        new PolePos(31, 28, 20, TrajectoryDistances.StackY, 0); //left

        public static PolePos CloseMidPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(31, 28, 20, TrajectoryDistances.StackY, 0) :
                        new PolePos(31, 28, 20, TrajectoryDistances.StackY, 0); //left


    }

    public static TrajectoryLocations TRAJECTORY_LOCATIONS = new TrajectoryLocations();


    /*
     * Util and Delays Vars ----------------------------------------------------------------------------
     */
    private double NextSlideHeightForStack = StackConeHeights[4];
    private int HeightIndex = 5;

    private void updateNextSlideHeight() {
        HeightIndex -= 1;
        NextSlideHeightForStack = StackConeHeights[HeightIndex];
    }

    private static class UtilAndDelays {


        public static double ResetPickupDelay = 0.2;
        public static double RotateArmDelay = 0.2;
        public static double LowerForPickupDelay = 1;

        public static double GiveUpDelay = 8;
        public static double PoleWait = 100;

        public static double LowPoleWait = 800;

        public static double LowPoleReleaseDelay = 150;
        public static double LowerSlideAmount = 16;
        public static double LowerSlideAmountLowPole = 3;

        public static double LowerSlideDelay = 400;
        public static double LowerSlideDelayLowPole = 800;

        public static double LowerSlideDelayHighPole = 500;

        public static double OpenClawDelay = 10;
        public static double OpenClawPickupDelay = 0;

        public static double CloseClawDelay = 500;

        public static double CloseClawDisplace = 1;
    }

    public static UtilAndDelays UTIL_AND_DELAYS = new UtilAndDelays();


    /*
     * Parking Vars --------------------------------------------------------------------------------
     */
    private static class ParkingPositions {
        public static double Pos1 = (SIDE == AutoSide.Right ? 12 : 57); //Red Signal Sleeve Position
        public static double Pos2 = 36; //Green Signal Sleeve Position

        public static double Pos3 = (SIDE == AutoSide.Right ? 57 : 12); //Blue Signal Sleeve Position

        public static double Y = -TrajectoryDistances.StackY;

        public static double Rot = FINAL_ROT;
    }

    public static ParkingPositions PARKING_POSITIONS = new ParkingPositions();

    private static int ParkingPosition = 2;

    /*
     * Scoring & Target Vars -----------------------------------------------------------------------
     */

    /**
     * The goal for auto, the preloaded cone is a given so the overall auto will be:
     * 1 + ConesToScore.Cones (3) = 4 total cones
     */
    protected static class ConesToScore {
        public static int Count = 3;

        public static TargetPole Pole0 = TargetPole.CloseHigh;
        public static TargetPole Pole1 = TargetPole.CloseHigh;
        public static TargetPole Pole2 = TargetPole.CloseHigh;
        public static TargetPole Pole3 = TargetPole.CloseHigh;

        //Extra cones: typically don't count unless cones to score is > 3
        public static TargetPole Pole4 = TargetPole.CloseHigh;
        public static TargetPole Pole5 = TargetPole.CloseHigh;

        public static TargetPole[] getPoles() { //Returns an array of the poles to score on
            return new TargetPole[]{
                    Pole0,
                    Pole1,
                    Pole2,
                    Pole3,
                    Pole4,
                    Pole5
            };
        }
    }

    private static class AlignmentTimes {
        public static boolean[] AlignStack = {true, true, true, true, true, true, true};
        public static boolean[] AlignPole = {true, true, true, true, true, true, true};
    }

    public static ConesToScore CONES_TO_SCORE = new ConesToScore();

    private int AlignStackIndex = 0;
    private int AlignPole = 0;
    private Pose2d TempPolePos = null;

    private Pose2d TempStackPos = new Pose2d(SIDE * TrajectoryLocations.StackPos.X, TrajectoryLocations.StackPos.Y, FINAL_ROT);

    private Pose2d TempLowPolePos = null;

    @Override
    public void runOpMode() throws InterruptedException {

        AutoAlignManager.CenterHeading = FINAL_ROT;

        /*
         * Init Vars -------------------------------------------------------------------------------
         */

        PoseStorage.reset();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Slide Slide = new Slide(hardwareMap.get(DcMotor.class, "slide"));

        Arm Arm = new Arm(hardwareMap.get(Servo.class, "arm"));
        Arm.setRotation(ArmRotation.Center);

        Claw Claw = new Claw(
                hardwareMap.get(Servo.class, "claw"),
                hardwareMap.get(DistanceSensor.class, "clawDistanceSensor"),
                hardwareMap.get(Servo.class, "poleBrace")
        );
        Claw.close();

        Servo podLift = hardwareMap.get(Servo.class, "podLift");
        podLift.setPosition(PodLiftPosition.Down);

        DistanceSensor LeftSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor RightSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        ColorSensor ColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Camera Camera = new Camera(hardwareMap);

        drive.setPoseEstimate(SIDE == AutoSide.Right ? AutoPositions.StartRight : AutoPositions.StartLeft);

        Slide.resetToZero();

        TrajectorySequence firstTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true) //reverse splines
                .splineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.FirstDrive.X, -60, Math.toRadians(270)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                ) //clear the wall
                .splineTo(new Vector2d(SIDE * TrajectoryLocations.FirstDrive.X, -46), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive around ground junction
                .splineTo(new Vector2d(SIDE * TrajectoryLocations.FirstDrive.X, TrajectoryLocations.FirstDrive.Y), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive to first high pole and turn
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.CloseHighPolePos.FindX, TrajectoryLocations.CloseHighPolePos.Y, Math.toRadians(FINAL_ROT)),
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                ) //drive to first high pole
                .build();


        /*
         * Init Actions -----------------------------------------------------------------------------
         */

        while (!isStarted() && !isStopRequested()) {
            Camera.scanForTags();
            telemetry.addData("Detected", "Tag " + Camera.getDetectedTag());
            telemetry.addLine("");
            telemetry.addData("Poles", Arrays.toString(ConesToScore.getPoles()));
            telemetry.update();
        }

        waitForStart();

        Claw.close();

        if (isStopRequested()) return;

        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        ParkingPosition = Camera.getDetectedTag();

        Claw.close();
        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

        drive.followTrajectorySequence( //drive away from wall and to high pole
                firstTraj
        );

        {
            TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.FastTurn, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                            )
                            .build()
            );
        }

        /*
         * Scoring 5 stack -------------------------------------------------------------------------
         */
        int ConesScored = 0;
        TargetPole[] TargetPoles;
        scoreCones:
        while (ConesScored < ConesToScore.Count) {
            TargetPoles = ConesToScore.getPoles();

            /*
             * Score Cone --------------------------------------------------------------------------
             */
            switch (TargetPoles[ConesScored]) {

                /*
                 * Score on far high pole ----------------------------------------------------------
                 */
                case CloseHigh: { //Scope for this case
                    TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;
                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);
                    Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Left : ArmRotation.Right);

                    boolean FoundPole = true;
                    ElapsedTime findTimer = new ElapsedTime();
//                    while (!drive.autoPlace(LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right) && opModeIsActive()) {
//                        telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
//                        telemetry.addLine("");
//                        telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
//                        telemetry.update();
//
//                        if (
//                                (findTimer.seconds() > UtilAndDelays.GiveUpDelay)
//                                        || (SIDE == AutoSide.Right ?
//                                        (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) :
//                                        (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX))
//                        ) { //if took too long or went too far, give up
//                            drive.followTrajectory(
//                                    drive.trajectoryBuilder(drive.getPoseEstimate())
//                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
//                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
//                                            ) //drive to pole estimate location
//                                            .build()
//                            );
//                            FoundPole = false;
//                            break;
//                        }
//                    }

                    if (TempPolePos != null && !ReadEveryTime) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(TempPolePos,
                                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                        ) //drive to pole estimate location from last time
                                        .build()
                        );
                        if (ReadEveryOther) TempPolePos = null;
                    } else {
                        AutoAlignManager.smartAlignReset();
                        while (!AutoAlignManager.smartAlign(drive, LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right) && opModeIsActive()) {
                            telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                            telemetry.addLine("");
                            telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
                            telemetry.update();
                            if (
                                    (findTimer.seconds() > UtilAndDelays.GiveUpDelay)
                                            || (SIDE == AutoSide.Right ?
                                            (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) :
                                            (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX))
                            ) { //if took too long or went too far, give up
                                drive.followTrajectory(
                                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                                ) //drive to pole estimate location
                                                .build()
                                );
                                FoundPole = false;
                                break;
                            }
                        }
                        TempPolePos = AutoAlignManager.getCalculatedPosition();
                    }

                    if (UtilAndDelays.PoleWait > 0) sleep((long) UtilAndDelays.PoleWait);

                    Slide.setHeight(SlideHeight.HighPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Mid);

                    sleep((long) UtilAndDelays.LowerSlideDelayHighPole);

                    /*
                     * Driving back to 5 stack -----------------------------------------------------------------
                     */

                    ConesScored++; // Increment cones scored

                    if (ConesScored >= ConesToScore.Count) {
                        Claw.setOpenAmount(ClawPosition.Open);
                        Claw.open();

//                        sleep((long) UtilAndDelays.OpenClawDelay);

                        if (SIDE == AutoSide.Right) {
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .strafeRight(TrajectoryDistances.ClearPole)
                                            .build()
                            );
                        } else {
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .strafeLeft(TrajectoryDistances.ClearPole)
                                            .build()
                            );
                        }
                        break scoreCones; // If all cones have been scored, break out of loop
                    }


                    updateNextSlideHeight(); // Update the next slide height
                    Claw.setOpenAmount(ClawPosition.PickupOpen);

//                    drive.followTrajectorySequence(
//                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                                    .addDisplacementMarker(TrajectoryDistances.ClearPole / 4, () -> {
////                                        Claw.setOpenAmount(ClawPosition.Open);
//                                        Claw.open();
//
////                                        sleep((long) UtilAndDelays.OpenClawDelay);
//
//                                        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Min);
//                                    })
//                                    .splineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y - TrajectoryDistances.ClearPole, Math.toRadians(FINAL_ROT)), Math.toRadians(FINAL_ROT),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    )
//                                    .addDisplacementMarker(() -> {
//                                        Arm.setRotation(ArmRotation.Center);
//                                        Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);
//                                        Claw.close();
//                                    })
//                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.AvoidByStackPos.X, TrajectoryLocations.AvoidByStackPos.Y), Math.toRadians(FINAL_ROT),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    )
//                                    .addDisplacementMarker(() -> {
//                                        Claw.setOpenAmount(ClawPosition.PickupOpen);
//                                        Claw.open();
//                                    })
//                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.AlignStackPos.X, TrajectoryLocations.AlignStackPos.Y), Math.toRadians(FINAL_ROT),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    )
////                                    .splineTo(new Vector2d(SIDE * (TrajectoryLocations.StackPos.X - TrajectoryDistances.ForwardPickup - TrajectoryDistances.PickupOffset), TrajectoryLocations.StackPos.Y), Math.toRadians(FINAL_ROT),
////                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
////                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
////                                    )
////                                    .splineTo(new Vector2d(SIDE * (TrajectoryLocations.StackPos.X - TrajectoryDistances.ForwardPickup), TrajectoryLocations.StackPos.Y), Math.toRadians(FINAL_ROT),
////                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
////                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
////                                    )
////                                    .waitSeconds(0.1)
////                                    .forward(TrajectoryDistances.ForwardPickup)
//                                    .build()
//                    );


                    drive.followTrajectorySequence(
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), TempStackPos.getY(), Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .addDisplacementMarker(() -> {
                                        Arm.setRotation(ArmRotation.Center);
                                        Claw.setOpenAmount(ClawPosition.Open);
                                        Claw.open();
//                                        Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Min);
                                    })
                                    .UNSTABLE_addTemporalMarkerOffset(UtilAndDelays.LowerForPickupDelay, () -> {
                                        Arm.setRotation(ArmRotation.Center);
                                        Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);
                                        Claw.setOpenAmount(ClawPosition.PickupOpen);
                                        Claw.close();
                                    })
//                                    .UNSTABLE_addTemporalMarkerOffset(UtilAndDelays.ResetPickupDelay, () -> { //waits for arm to move
//                                        Arm.setRotation(ArmRotation.Center);
//                                        Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);
//                                        Claw.close();
//                                    })
                                    .splineTo(new Vector2d(SIDE * TrajectoryLocations.AlignStackPos.X, TempStackPos.getY()), Math.toRadians(FINAL_ROT),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );

                    Claw.setOpenAmount(ClawPosition.PickupOpen);
                    Claw.open();

                    //Line up with stack using alignment with low pole

                    if (AlignmentTimes.AlignStack[AlignStackIndex]) {

                        AutoAlignManager.smartAlignReset();
                        while (!AutoAlignManager.smartAlign(drive, LeftSensor, RightSensor, Chassis.PoleAlign.Forward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Right : Chassis.PoleAlign.Left, true) && opModeIsActive()) {
                            telemetry.addData("Left Sensor", LeftSensor.getDistance(DistanceUnit.INCH));
                            telemetry.addLine("");
                            telemetry.addData("Right Sensor", RightSensor.getDistance(DistanceUnit.INCH));
                            telemetry.addLine("");
                            telemetry.update();
                        }

                        TempStackPos = AutoAlignManager.getCalculatedCenterPosition();

                        TempLowPolePos = AutoAlignManager.getCalculatedPosition();

//                    TempPolePos.plus(new Pose2d(0, AutoAlignManager.getCalculatedPosition().getY() - TrajectoryDistances.StackY, 0));

//                    drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), TrajectoryDistances.StackY, drive.getPoseEstimate().getHeading()));

                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .forward(Math.abs(TrajectoryDistances.ForwardPickupX - drive.getPoseEstimate().getX()))
                                        .build()
                        );

                    } else {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(TempStackPos)
                                        .build()
                        );

                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .forward(Math.abs(TrajectoryDistances.ForwardPickupX - drive.getPoseEstimate().getX()))
                                        .build()
                        );

                    }
                    AlignStackIndex++;


//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.StackPos.X - TrajectoryDistances.ForwardPickup, TrajectoryLocations.StackPos.Y, Math.toRadians(FINAL_ROT)),
//                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                                    )
//                                    .build()
//                    );

//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .forward(TrajectoryDistances.ForwardPickup)
//                                    .build()
//                    );


                } //End of scope for this case
                break; //end of delivering to far high pole


                /*
                 * Score on close mid pole ---------------------------------------------------------
                 */
                case CloseMid: {
                } //End of scope for this case
                break; //end of delivering to far high pole

                /*
                 * Score on close high pole ---------------------------------------------------------
                 */
                case FarHigh: {
                } //End of scope for this case
                break; //end of delivering to close high pole


            } //End of scoring cone on pole


//            /*
//             * Picking up cone
//             */
//            Claw.setOpenAmount(ClawPosition.PickupOpen);
//            Claw.open();
//
//            sleep((long) UtilAndDelays.OpenClawPickupDelay);

            Claw.close();
            sleep((long) UtilAndDelays.CloseClawDelay);

            /*
             * Driving to next pole
             */
            TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseHighPolePos;
            switch (TargetPoles[ConesScored]) {
                case CloseHigh:
                    TargetPolePos = TrajectoryLocations.CloseHighPolePos;
                    break;
                //end of close high
                case CloseMid:
                    TargetPolePos = TrajectoryLocations.CloseMidPolePos;
                    break;
                //end of close mid
                case FarHigh:
                    TargetPolePos = TrajectoryLocations.FarHighPolePos;
                    break;
                //end of far high
            } //end of drive to pole case
            Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max); //set slide height to high pole
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                            )
                            .build()
            );


        } //end of loop

//        Claw.close();
//        sleep((long) UtilAndDelays.CloseClawDelay);
//
//        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Max);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(TempLowPolePos)
//                        .build()
//        );
//
//        Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Right : ArmRotation.Left);
//
//        sleep((long) UtilAndDelays.LowPoleWait);
//
//        Claw.setOpenAmount(ClawPosition.PickupOpen);
//        Claw.open();
//
////        Slide.setHeight(SlideHeight.LowPole - UtilAndDelays.LowerSlideAmountLowPole, SlideSpeed.Max);
//
////        sleep((long) UtilAndDelays.LowerSlideDelayLowPole);
//
////        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Min);
//
//        sleep((long) UtilAndDelays.LowPoleReleaseDelay);
//
//        Arm.setRotation(ArmRotation.Center);

        /*
         * Parking ---------------------------------------------------------------------------------
         */

//        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
//        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
//        Claw.close(); //Closes claw to lower slide
        double ParkingX = SIDE * ParkingPositions.Pos2; //Sets parking position to the left
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                ParkingX = SIDE * ParkingPositions.Pos1;
                break;
            case 2:
                ParkingX = SIDE * ParkingPositions.Pos2;
                break;
            case 3:
                ParkingX = SIDE * ParkingPositions.Pos3;
                break;
        }
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addTemporalMarker(UtilAndDelays.ResetPickupDelay, () -> {
                            Claw.close();
                            Arm.setRotation(ArmRotation.Center); //Sets arm to center position
                            Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
                        })
                        .lineToLinearHeading(new Pose2d(ParkingX, TempStackPos.getY(), Math.toRadians(ParkingPositions.Rot)),
                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                        )
                        .build()
        );
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide

        /*
         * End of auto -----------------------------------------------------------------------------
         */

        telemetry.addData("Time Remaining", 30 - timer.seconds());
        telemetry.update();

        PoseStorage.CurrentPose = drive.getPoseEstimate();

        while (opModeIsActive()) { //Waits for autonomous to end
            idle();
        }
    }
}

