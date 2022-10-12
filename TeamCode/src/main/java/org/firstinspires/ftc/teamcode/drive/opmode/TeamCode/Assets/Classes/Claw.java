package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Claw class which contains all the methods for the claw of the robot
*/
public class Claw {
    
    private Servo ClawServo;
    
    /**
     * Creates a new claw with 1 servo
     * @param ClawServo The servo that will be used for the claw
    */
    public Claw(Servo ClawServo) {
        this.ClawServo = ClawServo;

        this.ClawServo.scaleRange(ClawPosition.Min, ClawPosition.Max); // Sets the range of the servo to ClawPosition min and max, ex: 0.0 to 1.0
    }

    /**
     * Sets the position of the servo
     * @param position The position to set the servo to as percentage 0-100
    */
    public void setPosition(double position) {
        this.ClawServo.setPosition(
            this.percentToPosition(position) // Converts the position to a percentage
        );
    }

    /**
     * Opens the claw
    */
    public void open() {
        this.setPosition(ClawPosition.Open);
    }

    /**
     * Closes the claw
    */
    public void close() {
        this.setPosition(ClawPosition.Close);
    }

    /**
     * Converts a percentage to a servo position
     * @param percent The percentage to convert as an int
     * @return The servo position as a double
    */
    private double percentToPosition(double percent) {
        return (double) (ClawPosition.Max - ClawPosition.Min) * percent / 100 + ClawPosition.Min;
    } 
 
}
