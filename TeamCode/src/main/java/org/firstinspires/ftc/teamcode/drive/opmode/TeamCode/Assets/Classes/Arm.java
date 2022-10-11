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

}
