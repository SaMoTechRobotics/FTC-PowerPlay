package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Arm class which contains all the methods for the arm of the robot
*/
public class Arm {
    
    private Servo ArmoServo;
    
    /**
     * Creates a new arm with 1 servo
     * @param ArmServo The servo that will be used for the arm
    */
    public Arm(Servo ArmServo) {
        this.ArmServo = ArmServo;

    }

    /**
     * Sets the position of the servo
     * @param position The position to set the servo to as rotation in degrees
    */
    public final void setRotation(double rotation) {
        this.ArmServo.setPosition(
            this.percentToPosition(rotation) // Converts the position to a percentage
        );
    }
    

    /**
     * Converts a percentage to a servo position
     * @param percent The percentage to convert as an int
     * @return The servo position as a double
    */
    private final double rotationToPosition(double rotation) {
        return (rotation - ArmPosition.Min) / (ArmPosition.Max - ArmPosition.Min);
    }
}
