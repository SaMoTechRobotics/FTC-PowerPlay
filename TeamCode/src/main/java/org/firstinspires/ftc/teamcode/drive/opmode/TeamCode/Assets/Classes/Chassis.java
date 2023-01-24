package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Chassis.PoseStorage;

/**
 * Chassis class which contains all the methods for the chassis of the robot
 */
public class Chassis {


    public Wheels Wheels;
    public DistanceSensor LeftSensor;
    public DistanceSensor RightSensor;
    public double DriveSpeed = ChassisSpeed.MidDrive;
    public double TurnSpeed = ChassisSpeed.MidTurn;
    public double StrafeSpeed = ChassisSpeed.MidStrafe;
    public boolean brake = true;
    /**
     * The stored autonomous class to control robot with RR
     */
    public SampleMecanumDrive MecanumDrive;
    public ChassisMode Mode = ChassisMode.Manual;


    /**
     * Creates a new chassis with 4 motors
     *
     * @param hardwareMap The hardware map of the robot
     */
    public Chassis(HardwareMap hardwareMap) {
        this.Wheels =
                new Wheels(
                        hardwareMap.get(DcMotor.class, "frontLeft"),
                        hardwareMap.get(DcMotor.class, "frontRight"),
                        hardwareMap.get(DcMotor.class, "backLeft"),
                        hardwareMap.get(DcMotor.class, "backRight")
                );

        this.LeftSensor =
                hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        this.RightSensor =
                hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        this.Wheels.FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.Wheels.BackLeft.setDirection(DcMotor.Direction.REVERSE);

        this.MecanumDrive = new SampleMecanumDrive(hardwareMap);
        this.MecanumDrive.setPoseEstimate(PoseStorage.CurrentPose);
        this.MecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the power of the motor provided
     *
     * @param motor The motor to set the power of
     * @param power The power to set the motors to, from -1.0 to 1.0
     */
    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    /**
     * Toggles the zero power behavior of the motors
     *
     * @param brakeOn Whether or not to brake on zero power
     */
    public void toggleBrake(boolean brakeOn) {
        this.brake = brakeOn;
        DcMotor.ZeroPowerBehavior behavior = brakeOn
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        this.Wheels.FrontLeft.setZeroPowerBehavior(behavior);
        this.Wheels.FrontRight.setZeroPowerBehavior(behavior);
        this.Wheels.BackLeft.setZeroPowerBehavior(behavior);
        this.Wheels.BackRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets the speed of the chassis based of the gamepad1 bumpers
     *
     * @param high Whether or not the high speed button is pressed, gamepad1 left bumper
     * @param low  Whether or not the low speed button is pressed, gamepad1 right bumper
     */
    public void updateSpeed(boolean high, boolean low) {
        if (high) { // Left bumper is max speeds
            this.DriveSpeed = ChassisSpeed.MaxDrive;
            this.TurnSpeed = ChassisSpeed.MaxTurn;
            this.StrafeSpeed = ChassisSpeed.MaxStrafe;
            this.toggleBrake(true);
        } else if (low) { // Right bumper is min speeds
            this.DriveSpeed = ChassisSpeed.MinDrive;
            this.TurnSpeed = ChassisSpeed.MinTurn;
            this.StrafeSpeed = ChassisSpeed.MinStrafe;
            this.toggleBrake(false);
        } else { // No bumper is mid speeds
            this.DriveSpeed = ChassisSpeed.MidDrive;
            this.TurnSpeed = ChassisSpeed.MidTurn;
            this.StrafeSpeed = ChassisSpeed.MidStrafe;
            this.toggleBrake(true);
        }
    }

//    /**
//     * Sets the power of all the motors based of the joysticks
//     *
//     * @param driveStick  The joystick for moving forward and backward, ex: left y
//     * @param strafeStick The joystick for strafing, ex: left x
//     * @param turnStick   The joystick for turning, ex: right x
//     */
//    public void updateWithControls(
//            double driveStick,
//            double strafeStick,
//            double turnStick,
//            GamepadEx gamepad,
//            Arm arm,
//            Claw claw
//    ) {
//        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP))
//            this.driveAlign = PoleAlign.Backward;
//        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
//            this.driveAlign = PoleAlign.Forward;
//        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
//            this.strafeAlign = PoleAlign.Right;
//        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
//            this.strafeAlign = PoleAlign.Left;
//
////        boolean align = gamepad.wasJustPressed(GamepadKeys.Button.A);
//        boolean align = false;
//
//        if (align)
//            this.Mode = ChassisMode.AutoPlace; // If align button is pressed, set mode to auto place
//        else if (driveStick != 0 || strafeStick != 0 || turnStick != 0) // If any of the sticks are not 0, set mode to manual
//            this.Mode = ChassisMode.Manual;
//        if (this.Mode == ChassisMode.Manual) {
//            this.MecanumDrive.breakFollowing(); // If in manual mode, end current autonomous movement
//            /*
//             * The different powers of the motors based of the joysticks
//             */
//            double frontLeftPower =
//                    (driveStick * this.DriveSpeed) +
//                            (turnStick * this.TurnSpeed) +
//                            (strafeStick * this.StrafeSpeed);
//            double frontRightPower =
//                    (driveStick * this.DriveSpeed) -
//                            (turnStick * this.TurnSpeed) -
//                            (strafeStick * this.StrafeSpeed);
//            double backLeftPower =
//                    (driveStick * this.DriveSpeed) +
//                            (turnStick * this.TurnSpeed) -
//                            (strafeStick * this.StrafeSpeed);
//            double backRightPower =
//                    (driveStick * this.DriveSpeed) -
//                            (turnStick * this.TurnSpeed) +
//                            (strafeStick * this.StrafeSpeed);
//
//            /*
//             * Sets the power of all the motors for manual drive
//             */
//            this.setPower(this.Wheels.FrontLeft, frontLeftPower);
//            this.setPower(this.Wheels.FrontRight, frontRightPower);
//            this.setPower(this.Wheels.BackLeft, backLeftPower);
//            this.setPower(this.Wheels.BackRight, backRightPower);
//        } else if (this.Mode == ChassisMode.AutoPlace) { // If in auto place mode, update autonomous movement
//            this.autoPlace(arm, claw, this.driveAlign, this.strafeAlign);
//        } else { // If something goes wrong, set mode to manual and stop all movement
//            this.Mode = ChassisMode.Manual;
//            this.setPower(this.Wheels.FrontLeft, 0);
//            this.setPower(this.Wheels.FrontRight, 0);
//            this.setPower(this.Wheels.BackLeft, 0);
//            this.setPower(this.Wheels.BackRight, 0);
//        }
//    }

    /**
     * Drives the robot based on the input from gamepad 1 and if auto drive is enabled
     *
     * @param gamepad1  The gamepad to use for input, (gamepad1)
     * @param autoDrive Whether or not auto drive is enabled
     */
    public void updateWithControls(
            GamepadEx gamepad1,
            boolean autoDrive
    ) {
        if (gamepad1.getLeftX() != 0 || gamepad1.getLeftY() != 0 || gamepad1.getRightX() != 0) {
            double driveStick = gamepad1.getLeftY();
            double strafeStick = gamepad1.getLeftX();
            double turnStick = gamepad1.getRightX();

            this.Mode = ChassisMode.Manual;
            this.MecanumDrive.breakFollowing();

            /*
             * The different powers of the motors based of the joysticks
             */
            double frontLeftPower =
                    (driveStick * this.DriveSpeed) +
                            (turnStick * this.TurnSpeed) +
                            (strafeStick * this.StrafeSpeed);
            double frontRightPower =
                    (driveStick * this.DriveSpeed) -
                            (turnStick * this.TurnSpeed) -
                            (strafeStick * this.StrafeSpeed);
            double backLeftPower =
                    (driveStick * this.DriveSpeed) +
                            (turnStick * this.TurnSpeed) -
                            (strafeStick * this.StrafeSpeed);
            double backRightPower =
                    (driveStick * this.DriveSpeed) -
                            (turnStick * this.TurnSpeed) +
                            (strafeStick * this.StrafeSpeed);

            /*
             * Sets the power of all the motors for manual drive
             */
            this.setPower(this.Wheels.FrontLeft, frontLeftPower);
            this.setPower(this.Wheels.FrontRight, frontRightPower);
            this.setPower(this.Wheels.BackLeft, backLeftPower);
            this.setPower(this.Wheels.BackRight, backRightPower);
        } else if (autoDrive && !gamepad1.getButton(GamepadKeys.Button.A) && // If auto drive is enabled and the escape auto button is not pressed
                (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP) || gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN) ||
                        gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT) || gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT))) { // If any of the dpad buttons are pressed
            this.Mode = ChassisMode.AutoDrive;
            this.autoDrive(gamepad1);
        } else if (autoDrive && !gamepad1.getButton(GamepadKeys.Button.A) && this.Mode == ChassisMode.AutoDrive) { // If already auto driving
            // Let the robot auto drive
        } else {
            this.Mode = ChassisMode.Manual;
            this.setPower(this.Wheels.FrontLeft, 0);
            this.setPower(this.Wheels.FrontRight, 0);
            this.setPower(this.Wheels.BackLeft, 0);
            this.setPower(this.Wheels.BackRight, 0);
        }
    }

    private void autoDrive(
            GamepadEx gamepad1
    ) {
        this.Mode = ChassisMode.AutoDrive;
        if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
            this.MecanumDrive.followTrajectoryAsync(
                    this.MecanumDrive.trajectoryBuilder(this.MecanumDrive.getPoseEstimate())
                            .forward(12)
                            .build()
            );
        }
    }

    /**
     * Returns the heading of the chassis rounded to the nearest 90 degrees
     *
     * @return The enum heading of the chassis
     */
    public ChassisHeading getRoundedHeading() {
        int roundedHeading = (int) Math.round(this.MecanumDrive.getPoseEstimate().getHeading() / 90) * 90;
        switch (roundedHeading) {
            case 270:
                //noinspection DuplicateBranchesInSwitch
                return ChassisHeading.Forward;
            case 90:
                return ChassisHeading.Back;
            case 180:
                return ChassisHeading.Left;
            case 0:
            case 360:
                return ChassisHeading.Right;
            default:
                return ChassisHeading.Forward;
        }
    }

    /**
     * Updates the position of the chassis and updates the RR instance of chassis that handles autonomous
     */
    public final void updatePosition() {
        this.MecanumDrive.update();
    }

    /**
     * Returns the current position of the chassis
     *
     * @return The current position of the chassis
     */
    public final Pose2d getPosition() {
        return this.MecanumDrive.getPoseEstimate();
    }

    /**
     * Returns the left distance sensors value in inches
     *
     * @return The left distance sensors value in inches
     */
    public final double getLeftDistance() {
        return this.LeftSensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Returns the right distance sensors value in inches
     *
     * @return The right distance sensors value in inches
     */
    public final double getRightDistance() {
        return this.RightSensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Enum for the current mode of the chassis
     */
    public enum ChassisMode {
        Manual, //Manual control, takes input from the joysticks
        AutoDrive, //Autonomous control, does current autonomous path or action, can be interrupted by manual control
    }

    public enum ChassisHeading {
        Forward,
        Left,
        Right,
        Back
    }

    /**
     * The motors of the chassis
     */
    private static class Wheels {
        public DcMotor FrontLeft;
        public DcMotor FrontRight;
        public DcMotor BackLeft;
        public DcMotor BackRight;

        /**
         * Constructor for the wheels
         *
         * @param frontLeft  The front left motor
         * @param frontRight The front right motor
         * @param backLeft   The back left motor
         * @param backRight  The back right motor
         */
        public Wheels(
                DcMotor frontLeft,
                DcMotor frontRight,
                DcMotor backLeft,
                DcMotor backRight
        ) {
            this.FrontLeft = frontLeft;
            this.FrontRight = frontRight;
            this.BackLeft = backLeft;
            this.BackRight = backRight;
        }
    }
}
