package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Chassis class which contains all the methods for the chassis of the robot
*/
public class Chassis {

    /**
     * The motors of the chassis
    */
    public class Wheels {
        public DcMotor FrontLeft;
        public DcMotor FrontRight;
        public DcMotor BackLeft;
        public DcMotor BackRight;

        /**
         * Constructor for the wheels
         * @param frontLeft The front left motor
         * @param frontRight The front right motor
         * @param backLeft The back left motor
         * @param backRight The back right motor
        */
        public Wheels(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
            this.FrontLeft = frontLeft;
            this.FrontRight = frontRight;
            this.BackLeft = backLeft;
            this.BackRight = backRight;
        }
    }

    public Wheels Wheels;

    public double DriveSpeed = ChassisSpeed.MidDrive;
    public double TurnSpeed = ChassisSpeed.MidTurn;
    public double StrafeSpeed = ChassisSpeed.MidStrafe;

    public boolean brake = false;

    /**
     * Creates a new chassis with 4 motors
     * @param FrontLeft
     * @param FrontRight
     * @param BackLeft
     * @param BackRight
    */
    public Chassis(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight) {
        this.Wheels = new Wheels(
            FrontLeft,
            FrontRight,
            BackLeft,
            BackRight
        );

        this.Wheels.FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.Wheels.BackLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets the power of the motor provided
     * @param motor The motor to set the power of
     * @param power The power to set the motors to, from -1.0 to 1.0
    */
    public void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    /**
     * Toggles the zero power behavior of the motors
     * @param brakeOn Whether or not to brake on zero power
     */
    public void toggleBrake(boolean brakeOn) {
        this.brake = brakeOn;
        DcMotor.ZeroPowerBehavior behavior = brakeOn ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        this.Wheels.FrontLeft.setZeroPowerBehavior(behavior);
        this.Wheels.FrontRight.setZeroPowerBehavior(behavior);
        this.Wheels.BackLeft.setZeroPowerBehavior(behavior);
        this.Wheels.BackRight.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets the power of all the motors based of the joysticks
     * @param driveStick The joystick for moving forward and backward, ex: left y
     * @param strafeStick The joystick for strafing, ex: left x
     * @param turnStick The joystick for turning, ex: right x
    */
    public void updateWithControls(double driveStick, double strafeStick, double turnStick) {
        double frontLeftPower = (driveStick * this.DriveSpeed) + (turnStick * this.TurnSpeed) + (strafeStick * this.StrafeSpeed);
        double frontRightPower = (driveStick * this.DriveSpeed) - (turnStick * this.TurnSpeed) - (strafeStick * this.StrafeSpeed);
        double backLeftPower = (driveStick * this.DriveSpeed) + (turnStick * this.TurnSpeed) - (strafeStick * this.StrafeSpeed);
        double backRightPower = (driveStick * this.DriveSpeed) - (turnStick* this.TurnSpeed) + (strafeStick * this.StrafeSpeed);

        this.setPower(this.Wheels.FrontLeft, frontLeftPower);
        this.setPower(this.Wheels.FrontRight, frontRightPower);
        this.setPower(this.Wheels.BackLeft, backLeftPower);
        this.setPower(this.Wheels.BackRight, backRightPower);
    }


    public void addTelemetry(telemetry telemetry) {
    }

    
}
