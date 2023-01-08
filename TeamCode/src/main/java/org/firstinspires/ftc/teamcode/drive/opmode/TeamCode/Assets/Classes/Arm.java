package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;

/**
 * Arm class which contains all the methods for the arm of the robot
 */
public class Arm {

    private final Servo ArmServo;

    /**
     * The target rotation of the arm
     */
    private double TargetRotation = ArmRotation.Center;

    private double RampingSpeed;

    /**
     * Creates a new arm with 1 servo
     *
     * @param ArmServo The servo that will be used for the arm
     */
    public Arm(Servo ArmServo) {
        this.ArmServo = ArmServo;
    }

    /**
     * Returns the rotation of the arm
     *
     * @return The position of the servo
     */
    public final double getRotation() {
        return this.ArmServo.getPosition();
    }

    /**
     * Sets the position of the servo
     *
     * @param rotation The position to set the servo to as rotation in degrees
     */
    public final void setRotation(double rotation) {
        this.TargetRotation = rotation;
//        if (this.ArmServo.getPosition() != this.rotationToPosition(rotation)) {
            this.ArmServo.setPosition(
                    this.rotationToPosition(rotation) // Converts the position to a percentage
            );
//        }
    }

    /**
     * Sets the target rotation of the arm
     * @param rotation The target rotation of the arm
     */
    public final void setTargetRotation(double rotation) {
        this.RampingSpeed = ArmRotation.MaxSpeed;
        this.TargetRotation = rotation;
    }

    /**
     * Returns the target rotation of the arm
     * @return The target rotation of the arm
     */
    public final double getTargetRotation() {
        return this.TargetRotation;
    }

    /**
     * Runs the arm to the target rotation
     * @param const_speed The speed to run the arm at (optional)
     */
    public final void runToTargetRotation(double... const_speed) {
        double speed = ArmRotation.MaxSpeed;
        if(const_speed.length > 0) {
            speed = const_speed[0];
        } else {
            speed = this.getRunSpeed(this.getTargetRotation(), this.getRotation());
        }
        if (this.getRotation() < this.TargetRotation) {
            this.ArmServo.setPosition(
                    this.rotationToPosition(this.getRotation() + speed) // Converts the position to a percentage
            );
        } else if (this.getRotation() > this.TargetRotation) {
            this.ArmServo.setPosition(
                    this.rotationToPosition(this.getRotation() - speed) // Converts the position to a percentage
            );
        }
    }

    /**
     * Returns the speed to run the arm at
     * @param target The target rotation of the arm
     * @param current The current rotation of the arm
     * @return The speed to run the arm at
     */
    public final double getRunSpeed(double target, double current) {
        double speed = ArmRotation.MaxSpeed;
        if (Math.abs(target - current) < ArmRotation.AccelMargin) {
//            if(this.RampingSpeed > ArmRotation.MinSpeed) {
//                this.RampingSpeed -= ArmRotation.AccelSpeed;
//            } else if(this.RampingSpeed < ArmRotation.MinSpeed) {
//                this.RampingSpeed += ArmRotation.AccelSpeed;
//            }
            speed = ArmRotation.MinSpeed;
//            speed = this.RampingSpeed;
        } else {
//            this.RampingSpeed = ArmRotation.MaxSpeed;
        }
        return speed;
    }

    /**
     * Updates the arm with controls
     *
     * @param speed  The speed to rotate the arm at from gamepad2 left stick y
     * @param left   The left bumper of gamepad2
     * @param right  The right bumper of gamepad2
     * @param center The y button of gamepad2
     */
    public final void updateWithControls(
            double speed,
            boolean left,
            boolean right,
            boolean center,
            boolean const_speed
    ) {
        if (center) {
//            if(const_speed)
                this.setRotation(ArmRotation.Center);
//            else this.setTargetRotation(ArmRotation.Center);
//            this.setRotation(ArmRotation.Center); // Sets the rotation to the middle
        } else if (left) {
//            if(const_speed)
                this.setRotation(ArmRotation.Left);
//                else this.setTargetRotation(ArmRotation.Left);
//            this.setRotation(ArmRotation.Left); // Sets the rotation to the left
        } else if (right) {
//            if(const_speed)
                this.setRotation(ArmRotation.Right);
//                else this.setTargetRotation(ArmRotation.Right);
//            this.setRotation(ArmRotation.Right); // Sets the rotation to the right
        } else {
            // this.addRotation(speed); // Adds the speed to the rotation
        }
//        if (!const_speed) this.runToTargetRotation();
    }

    /**
     * Converts a rotation to a servo position
     *
     * @param rotation The rotation to convert to servo position
     * @return The servo position from rotation
     */
    private final double rotationToPosition(double rotation) {
        return rotation;
    }
}
