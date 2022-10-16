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
    }

    public static double DriveSpeed = ChassisSpeed.MidDrive;
    public static double TurnSpeed = ChassisSpeed.MidTurn;
    public static double StrafeSpeed = ChassisSpeed.MidStrafe;

    /**
     * Creates a new chassis with 4 motors
     * @param FrontLeft
     * @param FrontRight
     * @param BackLeft
     * @param BackRight
    */
    public Chassis(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight) {
        this.Wheels.FrontLeft = FrontLeft;
        this.Wheels.FrontRight = FrontRight;
        this.Wheels.BackLeft = BackLeft;
        this.Wheels.BackRight = BackRight;
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
     * Sets the power of all the motors based of the joysticks
     * @param driveStick The joystick for moving forward and backward, ex: left y
     * @param strafeStick The joystick for strafing, ex: left x
     * @param turnStick The joystick for turning, ex: right x
    */
    public void updateWithControls(double driveStick, double strafeStick, double turnStick) {
        double frontLeftPower = (driveStick * ChassisSpeed.Drive) + (turnStick * ChassisSpeed.Turn) - (strafeStick * ChassisSpeed.Strafe);
        double frontRightPower = (driveStick * ChassisSpeed.Drive) + (turnStick * ChassisSpeed.Turn) + (strafeStick * ChassisSpeed.Strafe);
        double backLeftPower = (driveStick * ChassisSpeed.Drive) - (turnStick * ChassisSpeed.Turn) + (strafeStick * ChassisSpeed.Strafe);
        double backRightPower = (driveStick * ChassisSpeed.Drive) - (turnStick* ChassisSpeed.Turn) - (strafeStick * ChassisSpeed.Strafe);

        this.setPower(this.Wheels.FrontLeft, frontLeftPower);
        this.setPower(this.Wheels.FrontRight, frontRightPower);
        this.setPower(this.Wheels.BackLeft, backLeftPower);
        this.setPower(this.Wheels.BackRight, backRightPower);
    }


    
}
