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

    public void updateWithControls(double leftStickX, double leftStickY, double rightStickX) {
        double frontLeftPower = +((leftStickY * ChassisSpeed.Drive) + (rightStickX * ChassisSpeed.Turn) - (leftStickX * ChassisSpeed.Strafe));
        double frontRightPower = +((leftStickY * ChassisSpeed.Drive) + (rightStickX * ChassisSpeed.Turn) + (leftStickX * ChassisSpeed.Strafe));
        double backLeftPower = +((leftStickY * ChassisSpeed.Drive) - (rightStickX * ChassisSpeed.Turn) + (leftStickX * ChassisSpeed.Strafe));
        double backRightPower = +((leftStickY * ChassisSpeed.Drive) - (rightStickX * ChassisSpeed.Turn) - (leftStickX * ChassisSpeed.Strafe));

        setPower(this.Wheels.FrontLeft, frontLeftPower);
        setPower(this.Wheels.FrontRight, frontRightPower);
        setPower(this.Wheels.BackLeft, backLeftPower);
        setPower(this.Wheels.BackRight, backRightPower);
    }


    
}
