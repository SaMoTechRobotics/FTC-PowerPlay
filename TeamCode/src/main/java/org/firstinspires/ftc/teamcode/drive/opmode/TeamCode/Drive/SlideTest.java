package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SlideTest", group = "TeleOp")
public class SlideTest extends LinearOpMode {

    private DcMotor SlideMotor;
    private boolean SlideMoving = false;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        SlideMotor = hardwareMap.get(DcMotor.class, "slide");
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SlideMotor.setTargetPosition((int) SlideInfo.MaxTicks);

        // SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        // SlideMotor.setPower(0.2);

        // while (opModeIsActive() && SlideMotor.isBusy()) {
        //     if (gamepad1.a) {
        //         SlideMotor.setPower(SlideSpeeds.Stop);
        //     }

        //     telemetry.addData("Busy", SlideMotor.isBusy());
        //     telemetry.addData("Position", SlideMotor.getCurrentPosition());
        //     telemetry.addData("Target", SlideMotor.getTargetPosition());
        //     telemetry.addData("Power", SlideMotor.getPower());
        //     telemetry.addData("Mode", SlideMotor.getMode());
        //     telemetry.addData("Direction", SlideMotor.getDirection());

        //     telemetry.update();
        //     idle();
        // }

        // SlideMotor.setPower(SlideSpeeds.Stop);

        while (opModeIsActive()) {

            if (gamepad1.left_stick_y != 0) {
            //run slide with power
            SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Slide Power", gamepad1.left_stick_y * 0.5);
            SlideMotor.setPower(gamepad1.left_stick_y);
            } else {
                if(SlideMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    SlideMotor.setPower(0);
                }
            }

            if (gamepad1.dpad_up) {
            setSlide(SlideHeights.HighPole, SlideSpeeds.Max);
            }
            if (gamepad1.dpad_down) {
            setSlide(SlideHeights.LowPole, SlideSpeeds.Max);
            }
            if (gamepad1.dpad_left) {
            setSlide(SlideHeights.MidPole, SlideSpeeds.Max);
            }
            if (gamepad1.dpad_right) {
            setSlide(SlideHeights.Ground, SlideSpeeds.Max);
            }

            telemetry.addData("SlideHeight", SlideMotor.getCurrentPosition());
            
            if (SlideMotor.isBusy()) {
                telemetry.addData("SlideMotor", "Busy");
                SlideMoving = true;
            } else {
                telemetry.addData("SlideMotor", "Not Busy");
                SlideMoving = false;
            }
            
            telemetry.update();
        }
    }

    private void setSlide(double height, double speed) {
        // if (SlideMoving)
        //     return;

        double ticks = (height * 0.01) * SlideInfo.MaxTicks;
        telemetry.addData("SlideTicks", ticks);
        telemetry.update();

        SlideMotor.setTargetPosition((int) ticks);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setPower(speed);

        while (SlideMotor.isBusy()) {
            telemetry.addData("Slide Moving", SlideMotor.getCurrentPosition());
            telemetry.update();
        }

        SlideMotor.setPower(0);
        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
