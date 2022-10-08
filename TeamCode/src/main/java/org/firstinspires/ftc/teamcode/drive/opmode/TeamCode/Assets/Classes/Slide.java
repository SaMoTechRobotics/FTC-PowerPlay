package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Slide class which contains all the methods for the slide of the robot
*/
public class Slide {
    public DcMotor LeftMotor;
    public DcMotor RightMotor;

    /**
     * Creates a new slide with only 1 motor
     * @param MainMotor The motor that will be used for the slide as both left and right
    */     
    public Slide(DcMotor MainMotor) {
        this.LeftMotor = MainMotor;
        this.RightMotor = MainMotor;
    }

    /**
     * Creates a new slide with 2 motors
     * @param LeftMotor The left motor of the slide
     * @param RightMotor The right motor of the slide
    */
    public Slide(DcMotor LeftMotor, DcMotor RightMotor) {
        this.LeftMotor = LeftMotor;
        this.RightMotor = RightMotor;
    }

    /**
     * Sets the power of motors
     * @param power The power to set the motors to, from -1.0 to 1.0
    */
    public void setPower(double power) {
        this.LeftMotor.setPower(power);
        this.RightMotor.setPower(power);
    }

    /**
     * Sets the mode of the motors
     * @param mode The mode to set the motors to, from DcMotor.RunMode enum
    */
    public void setMode(DcMotor.RunMode mode) {
        this.LeftMotor.setMode(mode);
        this.RightMotor.setMode(mode);
    }

    /**
     * Sets the target position of the motors
     * @param target The target position in ticks
    */
    public void setTarget(int target) {
        this.LeftMotor.setTargetPosition(target);
        this.RightMotor.setTargetPosition(target);
    }
   
    /**
     * Sets the target position of the motors
     * @param height The target position as percentage
    */
    public void setHeight(int height, double speed) {
        int ticks = this.percentToTicks(heightPercentage);

        this.setTarget(ticks);
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.setPower(speed);

        this.setPower(SlideSpeed.Stop);
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Converts a percentage to ticks
     * @param height The target position as percentage
    */
    public int percentToTicks(int percent) {
        return (int)(((double) percent) / 100 * SlideInfo.MaxTicks);
    }
}
