package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Eyes.EyesPosition;


public class Eyes {

    private final Servo EyeServo;


    /**
     * Creates a new eye with 1 servo
     *
     * @param EyeServo the servo that controls the eye
     */
    public Eyes(Servo EyeServo) {
        this.EyeServo = EyeServo;
    }

    public final void setPosition(double position) {
        if (EyeServo.getPosition() != position) EyeServo.setPosition(position);
    }

    public final void syncWithSlide(int slideTicks) {
        EyeServo.setPosition(this.positionFromSlideTicks(slideTicks));
    }

    private double positionFromSlideTicks(double ticks) {
        double position = ticks * EyesPosition.SlideRatio;
        return Math.min(position, EyesPosition.MaxUp);
    }
}
