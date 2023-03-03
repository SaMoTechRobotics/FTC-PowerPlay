package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideHeight;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Slide.SlideSpeed;

/**
 * Slide class which contains all the methods for the slide of the robot
 */
public class Slide {

    private final DcMotor SlideMotor;
    private final double Speed = SlideSpeed.Max;
    public boolean isPaused = false;
    private double ManualSpeed = SlideSpeed.Min;
    private SlideStatus Status = SlideStatus.Stopped;
    private double LastSpeed = 0;
    private boolean GoingDown = false;

    private ElapsedTime GoingDownTimer = null;

    /**
     * Creates a new slide with only 1 motor
     *
     * @param SlideMotor The motor that will be used for the slide as both left and
     *                   right
     */
    public Slide(DcMotor SlideMotor) {
        this.SlideMotor = SlideMotor;

        this.SlideMotor.setDirection(DcMotor.Direction.REVERSE);
        // this.SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Status = SlideStatus.Stopped;
    }

    public final void resetToZero() {
        this.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public final void pause() {
        this.isPaused = true;
        this.LastSpeed = this.SlideMotor.getPower();
        this.setPower(0);
    }

    public final void resume() {
        this.isPaused = false;
        this.setPower(LastSpeed);
    }

    public final void stop() {
        this.Status = SlideStatus.Stopped;
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.setPower(0);
    }

    /**
     * Sets the mode of the motors
     *
     * @param mode The mode to set the motors to, from DcMotor.RunMode enum
     */
    public final void setMode(DcMotor.RunMode mode) {
        if (this.SlideMotor.getMode() != mode) this.SlideMotor.setMode(mode);
    }

    /**
     * Sets the target position of the motors
     *
     * @param target The target position in ticks
     */
    public final void setTarget(int target) {
        if (
                this.SlideMotor.getTargetPosition() != target
        ) this.SlideMotor.setTargetPosition(target);
    }

    /**
     * Gets the current position of the slide motor in ticks
     *
     * @return Ticks of slide motor
     */
    public final int getTicks() {
        return this.SlideMotor.getCurrentPosition();
    }

    public final double getInches() {
        return (
                (this.getTicks() / SlideHeight.TicksPerInch) +
                        SlideHeight.BaseHeight
        );
    }

    /**
     * @return returns the status of the slide
     */
    public final SlideStatus getStatus() {
        return this.Status;
    }

    /**
     * Sets the power of the slide motor based off the power from the joystick
     *
     * @param power power from joystick, ex: -1.0 to 1.1
     */
    public final void manualPower(double power) {
        if (
                (this.getTicks() < 0 && power < 0) ||
                        (this.getTicks() > this.inchesToTicks(SlideHeight.MaxHeight) && power > 0)
        ) return;
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.SlideMotor.setPower(power * this.ManualSpeed);
        this.Status = SlideStatus.ManualPower;
    }

    /**
     * Moves the slide to the target height
     *
     * @param height The target height in inches
     */
    public final void setHeight(double height, double speed) {
        int ticks = this.inchesToTicks(height);

        if (ticks < 0 || height > SlideHeight.MaxHeight) return;

        this.setTarget(ticks);
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.setPower(speed);

        this.Status = SlideStatus.MovingToTarget;
    }

    /**
     * Moves the slide to the target height asynchronously
     *
     * @param height The target height in inches
     */
    public final void setHeightAsync(double height, double speed) {
        int ticks = this.inchesToTicks(height);

        if (ticks < 0 || height > SlideHeight.MaxHeight) return;

        this.setTarget(ticks);
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.setPower(speed);

        this.Status = SlideStatus.MovingToTarget;

        while (this.getTicks() < this.SlideMotor.getTargetPosition()) {
            // wait for slide to finish
        }
    }

    /**
     * Holds the slide at its current position
     */
    public final void holdHeight() {
        if (this.Status == SlideStatus.Holding) return;
        if (this.getTicks() < SlideHeight.GoingDownGroundMargin) {
            return;
        }

        this.setTarget(this.getTicks());
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.setPower(SlideSpeed.Hold);

        this.Status = SlideStatus.Holding;
    }

    public final double getPower() {
        return this.SlideMotor.getPower();
    }

    /**
     * Sets the power of motors
     *
     * @param power The power to set the motors to, from -1.0 to 1.0
     */
    public final void setPower(double power) {
        if (this.SlideMotor.getPower() != power) this.SlideMotor.setPower(power);
    }

    /**
     * Updates the slide with controls
     *
     * @param power The power to set the motors to, from -1.0 to 1.0 from the gamepad2 left stick y
     * @param up    The up button on the dpad
     * @param left  The left button on the dpad
     * @param down  The down button on the dpad
     * @param right The right button on the dpad
     */
    public final void updateWithControls(
            double power,
            boolean up,
            boolean left,
            boolean down,
            boolean right,
            boolean AButton,
            Arm arm,
            Claw claw
    ) {
        if (up | down | left | power != 0) this.GoingDown = false;
//        if (this.GoingDown && (arm.getRotation() > ArmRotation.Center - ArmRotation.Margin && arm.getRotation() < ArmRotation.Center + ArmRotation.Margin)) {
        if (this.GoingDown && this.GoingDownTimer != null && this.GoingDownTimer.seconds() > ClawPosition.GoingDownTimeout) {
            this.GoingDownTimer = null;
            this.setHeight(SlideHeight.Ground, this.Speed);
            claw.close();
        } else if (this.GoingDown && this.GoingDownTimer == null) {
            if (this.getTicks() < SlideHeight.GoingDownGroundMargin) claw.open();
            this.GoingDown = false;
        }
        if (up)
            this.setHeight(SlideHeight.HighPole, this.Speed); // Slide set to high pole height if dpad up is pressed
        else if (left)
            this.setHeight(SlideHeight.MidPole, this.Speed); // Slide set to mid pole height if dpad left is pressed
        else if (down)
            this.setHeight(SlideHeight.LowPole, this.Speed); // Slide set to low pole height if dpad down is pressed
        else if (right) {
            this.GoingDown = true;
            arm.setRotation(ArmRotation.Center);
            this.GoingDownTimer = new ElapsedTime();
//            claw.close();
//            claw.raisePoleBrace();
//            this.setHeight(SlideHeight.Ground, this.Speed); // Slide set to ground height if dpad right is pressed
        } else if (power != 0) {
            this.manualPower(power); // Slide set to power from gamepad2 left stick y if no dpad buttons are pressed
//        } else if (this.getTicks() < SlideHeight.GroundMargin) {
//            if (this.Status != SlideStatus.Stopped) {
//            this.stop();
        } else if (this.getTicks() < SlideHeight.AutoClawMargin) {
            if (!claw.detectedCone() && !AButton) claw.open();
//            }
        } else if (
                this.Status != SlideStatus.Stopped &&
                        (this.Status == SlideStatus.ManualPower || this.atTargetPosition())
        ) {
            this.holdHeight(); // Slide set to hold height if no dpad buttons are pressed and the slide is not moving
        }
        if (up || down || left || right) return;
        if (this.SlideMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION && this.getTicks() <= 0 && this.SlideMotor.getPower() < 0)
            this.setPower(0);
        else if (this.getTicks() <= -10 && this.SlideMotor.getPower() != 0) this.setPower(0);
    }

    public final void waitForArm(double armRotation) {
        if (SlideHeight.WaitForArm && this.SlideMotor.getTargetPosition() < this.SlideMotor.getCurrentPosition() && this.SlideMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (armRotation > ArmRotation.Center - ArmRotation.Margin && armRotation < ArmRotation.Center + ArmRotation.Margin) {
                this.SlideMotor.setPower(this.Speed);
            } else {
                this.SlideMotor.setPower(0);
            }
        }
    }


    public final void updateSpeed(boolean fast) {
        this.ManualSpeed = fast ? SlideSpeed.Mid : SlideSpeed.Min;
    }

    /**
     * @return boolean based of if slide is at target position, will always return false if the slide is manually moving
     */
    public final boolean atTargetPosition() {
        return this.Status != SlideStatus.ManualPower && this.getTicks() == this.SlideMotor.getTargetPosition();
    }

    /**
     * Checks if slide has cleared the safety margin for the arm to move
     *
     * @return True if slide has cleared the safety margin, false if not
     */
    public final boolean safeHeight() {
        return this.getTicks() > this.inchesToTicks(SlideHeight.SafetyHeight);
    }

    /**
     * Converts inches to ticks
     *
     * @param height The target position as inches
     */
    private final int inchesToTicks(double height) {
        return (int) (height * (SlideHeight.TicksPerInch - SlideHeight.BaseHeight));
    }

    public enum SlideStatus {
        MovingToTarget,
        ManualPower,
        Holding,
        Paused,
        Stopped,
    }
}
