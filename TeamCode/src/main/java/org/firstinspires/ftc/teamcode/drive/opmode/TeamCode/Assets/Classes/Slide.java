package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;



public class Slide {
    public static double MaxTicks = -2000;
    public static double MinTicks = 0;

    
    public static void set(int heightPercentage, double speed) {
        int ticks = Slide.percentToTicks(heightPercentage);

        SlideMotor.setTargetPosition(ticks);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setPower(speed);

        SlideMoving = true;

        while (SlideMotor.isBusy()) {
            telemetry.addData("Slide Moving", SlideMotor.getCurrentPosition());
            telemetry.update();
        }

        SlideMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMoving = false;
    }

    public static int percentToTicks(int heightPercentage) {
        return (int)(((double) heightPercentage) / 100 * SlideInfo.MaxTicks);
    }
}
