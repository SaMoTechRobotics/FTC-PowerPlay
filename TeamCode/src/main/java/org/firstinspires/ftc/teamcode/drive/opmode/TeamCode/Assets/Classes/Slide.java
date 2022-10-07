package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Slide {
    public DcMotor LeftMotor;
    public DcMotor RightMotor;

    public Slide(DcMotor LeftMotor, DcMotor RightMotor) {
        this.LeftMotor = LeftMotor;
        this.RightMotor = RightMotor;
    }


    public void setPower(double power) {
        this.LeftMotor.setPower(power);
        this.RightMotor.setPower(power);
    }
   
    public void setHeight(int heightPercentage, double speed) {
        int ticks = this.percentToTicks(heightPercentage);

        this.LeftMotor.setTargetPosition(ticks);
        this.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.LeftMotor.setPower(speed);


        while (this.LeftMotor.isBusy()) {
            telemetry.addData("Slide Moving", SlideMotor.getCurrentPosition());
            telemetry.update();
        }

        this.LeftMotor.setPower(0);
        this.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int percentToTicks(int heightPercentage) {
        return (int)(((double) heightPercentage) / 100 * SlideInfo.MaxTicks);
    }
}
