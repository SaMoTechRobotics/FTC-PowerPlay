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

    }

    /**
     * Sets the position of the servo
     * @param position The position to set the servo to as percentage 0-100
    */
    public final void setPosition(double position) {
        this.ClawServo.setPosition(
            this.percentToPosition(position) // Converts the position to a percentage
        );
    }

    /**
     * Opens the claw
    */
    public final void open() {
        this.setPosition(ClawPosition.Open);
    }

    /**
     * Closes the claw
    */
    public final void close() {
        this.setPosition(ClawPosition.Close);
    }

    /**
     * Toggles the claw
     * @param open The open position of the claw
     */
    public final void toggleOpen(boolean open) {
        if (open) {
            this.open();
        } else {
            this.close();
        }
    }

    /**
     * Converts a percentage to a servo position
     * @param percent The percentage to convert as an int
     * @return The servo position as a double
    */
    private final double percentToPosition(double percent) {
        return (double) (percent / 100);
    } 
 
}
