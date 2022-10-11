package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Slide class which contains all the methods for the slide of the robot
*/
public class Slide {
    private DcMotor SlideMotor;

    /**
     * Creates a new slide with only 1 motor
     * @param SlideMotor The motor that will be used for the slide as both left and right
    */     
    public Slide(DcMotor SlideMotor) {
        this.SlideMotor = SlideMotor;
    }

    /**
     * Sets the power of motors
     * @param power The power to set the motors to, from -1.0 to 1.0
    */
    public void setPower(double power) {
        this.SlideMotor.setPower(power);
    }

    /**
     * Sets the mode of the motors
     * @param mode The mode to set the motors to, from DcMotor.RunMode enum
    */
    public void setMode(DcMotor.RunMode mode) {
        this.SlideMotor.setMode(mode);
    }

    /**
     * Sets the target position of the motors
     * @param target The target position in ticks
    */
    public void setTarget(int target) {
        this.SlideMotor.setTargetPosition(target);
    }
   
    /**
     * Sets the target position of the motors
     * @param height The target position as percentage
    */
    public void setHeight(int height, double speed) {
        int ticks = this.percentToTicks(height);

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
    private int percentToTicks(int percent) {
        return (int)(((double) percent) / 100 * SlideInfo.MaxTicks);
    }
}
