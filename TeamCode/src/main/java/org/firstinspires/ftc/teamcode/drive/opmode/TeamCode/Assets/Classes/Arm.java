package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm class which contains all the methods for the arm of the robot
*/
public class Arm {
    
    private Servo ArmServo;
    
    /**
     * Creates a new arm with 1 servo
     * @param ArmServo The servo that will be used for the arm
    */
    public Arm(Servo ArmServo) {
        this.ArmServo = ArmServo;

        this.ArmServo.scaleRange(ArmRotation.Min, ArmRotation.Max);
    }

    /**
     * Sets the position of the servo
     * @param position The position to set the servo to as rotation in degrees
    */
    public final void setRotation(double rotation) {
        this.ArmServo.setPosition(
            this.rotationToPosition(rotation) // Converts the position to a percentage
        );
    }
    

    /**
     * Converts a rotation to a servo position
     * @param percent The rotation to convert to servo position
     * @return The servo position from rotation
    */
    private final double rotationToPosition(double rotation) {
        return rotation;
    }
}
