package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Pro;

import static org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight.StackConeHeights;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Arm;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Chassis;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Claw;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes.Slide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.PodLift.PodLiftPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Sensor.SensorColors;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoPositions;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.AutoSide;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.TargetPole;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Auto.Constants.Vec2;

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

    /*
     * Signal Sleeve Detection Vars ----------------------------------------------------------------
     */
    private static class DetectionParams {
        public static double StartingDist = 17;
        public static double DetectAttempts = 10;
        public static double DetectAttemptMultiplier = 0.25;
    }

    public static DetectionParams DETECTION_PARAMS = new DetectionParams();

    /*
     * Trajector Speed Vars -----------------------------------------------------------------------
     */

    private static class TrajectorySpeeds {
        public static double MaxSpeed = 90;
        public static double FastSpeed = 85;
        public static double NormalSpeed = 55;

        public static double MaxAccel = 60;
        public static double FastAccel = 60;
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
        public static double ClearPoleStrafe = 3.5;
        public static double DetectionDrive = 30;

        public static double ClearConeDrive = 17;

        public static double PickupForward = 8.5;
    }

    public static TrajectoryDistances TRAJECTORY_DISTANCES = new TrajectoryDistances();


    /*
     * Position Vars -------------------------------------------------------------------------------
     */

    private static class TrajectoryLocations {

        public static Vec2 FirstFindStackPos = new Vec2(35, -13);

        public static Vec2 StackPos = SIDE == AutoSide.Right ? new Vec2(56, -12.5) : new Vec2(56, -12);

        public static Vec2 FirstMidPolePos =
                SIDE == AutoSide.Right ?
                        new Vec2(33.5, -28.5) :
                        new Vec2(33, -27.5); //left

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
                        new PolePos(32.5, 31, 31, -10, 0) :
                        new PolePos(32.5, 31, 31, -10, 0); //left
        public static PolePos CloseMidPolePos =
                SIDE == AutoSide.Right ?
                        new PolePos(32, 31, 31, -13.5, 0) :
                        new PolePos(32, 31, 31, -13.5, 0); //left

    }

    public static TrajectoryLocations TRAJECTORY_LOCATIONS = new TrajectoryLocations();


    /*
     * Util and Delays Vars ----------------------------------------------------------------------------
     */
    private double NextSlideHeightForStack = StackConeHeights[4];
    private int HeightIndex = 4;

    private void updateNextSlideHeight() {
        HeightIndex -= 1;
        NextSlideHeightForStack = StackConeHeights[HeightIndex];
    }

    private static class UtilAndDelays {

        public static double GiveUpDelay = 4;
        public static double FirstPoleWait = 300;
        public static double PoleWait = 200;
        public static double LowerSlideAmount = 8;

        public static double LowerSlideDelay = 300;

        public static double LowerSlideDelayHighPole = 400;

        public static double OpenClawDelay = 10;
        public static double OpenClawPickupDelay = 0;

        public static double CloseClawDelay = 100;
    }

    public static UtilAndDelays UTIL_AND_DELAYS = new UtilAndDelays();


    /*
     * Parking Vars --------------------------------------------------------------------------------
     */
    private static class ParkingPositions {
        public static double Pos1 = (SIDE == AutoSide.Right ? 12 : 59); //Red Signal Sleeve Position
        public static double Pos2 = 36; //Green Signal Sleeve Position

        public static double Pos3 = (SIDE == AutoSide.Right ? 59 : 12); //Blue Signal Sleeve Position

        public static double Y = -13.5;

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
    private static class ConesToScore {
        public static int Count = 3;


        public static TargetPole Pole1 = TargetPole.FarHigh;
        public static TargetPole Pole2 = TargetPole.CloseHigh;
        public static TargetPole Pole3 = TargetPole.CloseHigh;

        //Extra cones: typically don't count unless cones to score is > 3
        public static TargetPole Pole4 = TargetPole.CloseMid;
        public static TargetPole Pole5 = TargetPole.CloseMid;

        public static TargetPole[] getPoles() { //Returns an array of the poles to score on
            return new TargetPole[]{
                    Pole1,
                    Pole2,
                    Pole3,
                    Pole4,
                    Pole5
            };
        }
    }

    public static ConesToScore CONES_TO_SCORE = new ConesToScore();

    @Override
    public void runOpMode() throws InterruptedException {

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

        drive.setPoseEstimate(SIDE == AutoSide.Right ? AutoPositions.StartRight : AutoPositions.StartLeft);

        Slide.resetToZero();

        //Timer that will be used to time the autonomous
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        /*
         * Init Actions -----------------------------------------------------------------------------
         */

        while (!isStarted()) {
            telemetry.addData("CloseHighPolePos", TrajectoryLocations.CloseHighPolePos.X);
            telemetry.addData("Poles", Arrays.toString(ConesToScore.getPoles()));
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Claw.close();
        Arm.setRotation(ArmRotation.Center);
        Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);

        /*
         * Signal Sleeve Detection -----------------------------------------------------------------
         */

        final boolean[] GotColor = {false};
        TrajectoryBuilder FullSpeedDetectionTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(TrajectoryDistances.DetectionDrive,
                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                );
        for (int i = 0; i < DetectionParams.DetectAttempts; i++) {
            FullSpeedDetectionTraj.addDisplacementMarker(DetectionParams.StartingDist + (i * DetectionParams.DetectAttemptMultiplier), () -> { //Reads signal sleeve
                if (GotColor[0] || ColorSensor.alpha() < SensorColors.AlphaDetectionMargin) {
                    if (!GotColor[0]) {
                        telemetry.addData("Alpha", ColorSensor.alpha());
                        telemetry.update();
                    }
                    return;
                }

                SensorColors.Color detectedColor = SensorColors.detectColor(ColorSensor);

                ParkingPosition = SensorColors.getParkingPosition( // reads parking position based of detected color
                        detectedColor
                );

                GotColor[0] = true;

                telemetry.addData("Detected Color", detectedColor);
                telemetry.addData("Parking Position", ParkingPosition);
                telemetry.update();
            });
        }

        drive.followTrajectory(FullSpeedDetectionTraj.build());

        /*
         * Scoring 1st Cone on Mid Pole ------------------------------------------------------------
         */

        Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Right : ArmRotation.Left); //Sets arm to left or right position

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.FirstMidPolePos.X, TrajectoryLocations.FirstMidPolePos.Y, Math.toRadians(270)),
                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                        )
                        .build()
        );

        sleep((long) UtilAndDelays.FirstPoleWait);

        Slide.setHeight(SlideHeight.MidPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Mid);

        sleep((long) UtilAndDelays.LowerSlideDelay);

        Claw.setOpenAmount(ClawPosition.Open);
        Claw.open();

        sleep((long) UtilAndDelays.OpenClawDelay);

        Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Mid);

        if (SIDE == AutoSide.Right) {
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeLeft(TrajectoryDistances.ClearPoleStrafe)
                            .build()
            );
        } else {
            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeRight(TrajectoryDistances.ClearPoleStrafe)
                            .build()
            );
        }

        /*
         * Driving toward 5 stack ------------------------------------------------------------------
         */

        Claw.close();
        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.LowPole, SlideSpeed.Mid);

        drive.followTrajectorySequence(
                drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(TrajectoryDistances.ClearConeDrive,
                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                        )
                        .addDisplacementMarker(() -> {
                            Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);
                        })
                        .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.FirstFindStackPos.X, TrajectoryLocations.FirstFindStackPos.Y, Math.toRadians(FINAL_ROT)),
                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                        )
//                        .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.StackPos.X, TrajectoryLocations.StackPos.Y, Math.toRadians(FINAL_ROT)),
//                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
//                        )
                        .build()
        );
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.StackPos.X, TrajectoryLocations.StackPos.Y, Math.toRadians(FINAL_ROT)),
                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                        )
                        .build()
        );

        /*
         * Scoring 5 stack -------------------------------------------------------------------------
         */

        int ConesScored = 0;
        TargetPole[] TargetPoles;
        while (ConesScored < ConesToScore.Count) {
            TargetPoles = ConesToScore.getPoles();

            Claw.setOpenAmount(ClawPosition.PickupOpen);
            Claw.open();

            sleep((long) UtilAndDelays.OpenClawPickupDelay);

            /*
             * Pickup Cone -------------------------------------------------------------------------
             */

            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(TrajectoryDistances.PickupForward,
                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                            )
                            .addDisplacementMarker(Claw::close)
                            .build()
            );

            Claw.close();
            sleep((long) UtilAndDelays.CloseClawDelay);

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

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );
                    Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Left : ArmRotation.Right);
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                    )
                                    .build()
                    );

                    boolean FoundPole = true;
                    ElapsedTime findTimer = new ElapsedTime();
                    while (!drive.autoPlace(LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Left : Chassis.PoleAlign.Right) && opModeIsActive()) {
                        if (findTimer.seconds() > UtilAndDelays.GiveUpDelay) {
                            FoundPole = false;
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );
                            break;
                        }
                        if (SIDE == AutoSide.Right ? (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) : (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX)) {
                            FoundPole = false;

                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );

                            break;
                        }
                    }

                    if (FoundPole && TargetPolePos.Adjust != 0) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .back(TargetPolePos.Adjust,
                                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                        )
                                        .build()
                        );
                    }

                    sleep((long) UtilAndDelays.PoleWait);

                    Slide.setHeight(SlideHeight.HighPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Mid);

                    sleep((long) UtilAndDelays.LowerSlideDelayHighPole);

                    Claw.setOpenAmount(ClawPosition.Open);
                    Claw.open();

                    sleep((long) UtilAndDelays.OpenClawDelay);

                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid);

                    if (SIDE == AutoSide.Right) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeRight(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    } else {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeLeft(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    }
                } //End of scope for this case
                break; //end of delivering to far high pole


                /*
                 * Score on close mid pole ---------------------------------------------------------
                 */
                case CloseMid: { //Scope for this case
                    TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.CloseMidPolePos;
                    Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Max);

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );
                    Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Right : ArmRotation.Left);
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                    )
                                    .build()
                    );

                    boolean FoundPole = true;
                    ElapsedTime findTimer = new ElapsedTime();
                    while (!drive.autoPlace(LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Right : Chassis.PoleAlign.Left) && opModeIsActive()) {
                        if (findTimer.seconds() > UtilAndDelays.GiveUpDelay) {
                            FoundPole = false;
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );
                            break;
                        }
                        if (SIDE == AutoSide.Right ? (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) : (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX)) {
                            FoundPole = false;

                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );

                            break;
                        }
                    }

                    if (FoundPole && TargetPolePos.Adjust != 0) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .back(TargetPolePos.Adjust,
                                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                        )
                                        .build()
                        );
                    }

                    sleep((long) UtilAndDelays.PoleWait);

                    Slide.setHeight(SlideHeight.MidPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Mid);

                    sleep((long) UtilAndDelays.LowerSlideDelay);

                    Claw.setOpenAmount(ClawPosition.Open);
                    Claw.open();

                    sleep((long) UtilAndDelays.OpenClawDelay);

                    Slide.setHeight(SlideHeight.MidPole, SlideSpeed.Mid);

                    if (SIDE == AutoSide.Right) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeLeft(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    } else {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeRight(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    }
                } //End of scope for this case
                break; //end of delivering to far high pole

                /*
                 * Score on close high pole ---------------------------------------------------------
                 */
                case FarHigh: { //Scope for this case
                    TrajectoryLocations.PolePos TargetPolePos = TrajectoryLocations.FarHighPolePos;
                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Max);

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                                    )
                                    .build()
                    );
                    Arm.setRotation(SIDE == AutoSide.Right ? ArmRotation.Right : ArmRotation.Left);
                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.FindX, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                            SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                    )
                                    .build()
                    );

                    boolean FoundPole = true;
                    ElapsedTime findTimer = new ElapsedTime();
                    while (!drive.autoPlace(LeftSensor, RightSensor, Chassis.PoleAlign.Backward, SIDE == AutoSide.Right ? Chassis.PoleAlign.Right : Chassis.PoleAlign.Left) && opModeIsActive()) {
                        if (findTimer.seconds() > UtilAndDelays.GiveUpDelay) {
                            FoundPole = false;
                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );
                            break;
                        }
                        if (SIDE == AutoSide.Right ? (drive.getPoseEstimate().getX() < SIDE * TargetPolePos.GiveUpX) : (drive.getPoseEstimate().getX() > SIDE * TargetPolePos.GiveUpX)) {
                            FoundPole = false;

                            drive.followTrajectory(
                                    drive.trajectoryBuilder(drive.getPoseEstimate())
                                            .lineToLinearHeading(new Pose2d(SIDE * TargetPolePos.X, TargetPolePos.Y, Math.toRadians(FINAL_ROT)),
                                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                            )
                                            .build()
                            );

                            break;
                        }
                    }

                    if (FoundPole && TargetPolePos.Adjust != 0) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .back(TargetPolePos.Adjust,
                                                SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.NormalSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.NormalAccel)
                                        )
                                        .build()
                        );
                    }

                    sleep((long) UtilAndDelays.PoleWait);

                    Slide.setHeight(SlideHeight.HighPole - UtilAndDelays.LowerSlideAmount, SlideSpeed.Mid);

                    sleep((long) UtilAndDelays.LowerSlideDelayHighPole);

                    Claw.setOpenAmount(ClawPosition.Open);
                    Claw.open();

                    sleep((long) UtilAndDelays.OpenClawDelay);

                    Slide.setHeight(SlideHeight.HighPole, SlideSpeed.Mid);

                    if (SIDE == AutoSide.Right) {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeLeft(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    } else {
                        drive.followTrajectory(
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .strafeRight(TrajectoryDistances.ClearPoleStrafe)
                                        .build()
                        );
                    }
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(drive.getPoseEstimate())
//                                    .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), TrajectoryLocations.StackPos.Y, Math.toRadians(FINAL_ROT)))
//                                    .build()
//                    );
                } //End of scope for this case
                break; //end of delivering to close high pole


            } //End of scoring cone on pole

            /*
             * Driving back to 5 stack -----------------------------------------------------------------
             */

            Arm.setRotation(ArmRotation.Center);

            ConesScored++; // Increment cones scored

            if (ConesScored >= ConesToScore.Count)
                break; // If all cones have been scored, break out of loop

            updateNextSlideHeight(); // Update the next slide height
            telemetry.addData("NextSlideHeight", NextSlideHeightForStack);
            telemetry.addData("SlideHeight Index", HeightIndex);
            telemetry.update();

            Slide.setHeight(NextSlideHeightForStack, SlideSpeed.Max);


            drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                            .addTemporalMarker(0.2, () -> {
                                Claw.setOpenAmount(ClawPosition.PickupOpen);
                                Claw.close();
                                Claw.open();
                            })
                            .lineToLinearHeading(new Pose2d(SIDE * TrajectoryLocations.StackPos.X, TrajectoryLocations.StackPos.Y, Math.toRadians(FINAL_ROT)),
                                    SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.FastSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                    SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.FastAccel)
                            )
                            .build());

        }

        /*
         * Parking ---------------------------------------------------------------------------------
         */

        Arm.setRotation(ArmRotation.Center); //Sets arm to center position
        Slide.setHeight(SlideHeight.Ground, SlideSpeed.Max); //Sets slide to ground height
        Claw.close(); //Closes claw to lower slide
        switch (ParkingPosition) { //Drives to parking position
            case 1:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos1, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
            case 2:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos2, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
            case 3:
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(SIDE * ParkingPositions.Pos3, ParkingPositions.Y, Math.toRadians(ParkingPositions.Rot)),
                                        SampleMecanumDrive.getVelocityConstraint(TrajectorySpeeds.MaxSpeed, TrajectorySpeeds.NormalTurn, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(TrajectorySpeeds.MaxAccel)
                                )
                                .build()
                );
                break;
        }
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

