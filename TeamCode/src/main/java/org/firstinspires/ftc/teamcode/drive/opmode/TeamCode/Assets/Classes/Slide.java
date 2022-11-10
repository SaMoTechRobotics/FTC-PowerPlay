package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Slide class which contains all the methods for the slide of the robot
 */
public class Slide {

  public static enum SlideStatus {
    MovingToTarget,
    ManualPower,
    Holding,
    Stopped,
  }

  private DcMotor SlideMotor;

  private double Speed = SlideSpeed.Max;
  private SlideStatus Status = SlideStatus.Stopped;

  /**
   * Creates a new slide with only 1 motor
   *
   * @param SlideMotor The motor that will be used for the slide as both left and
   *                   right
   */
  public Slide(DcMotor SlideMotor) {
    this.SlideMotor = SlideMotor;

    this.SlideMotor.setDirection(DcMotor.Direction.REVERSE);
    this.SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.Status = SlideStatus.Stopped;
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
   * @return Ticks of slide motor
   */
  public final int getTicks() {
    return this.SlideMotor.getCurrentPosition();
  }

  public final double getInches() {
    return (
      (double) (this.getTicks() / SlideHeight.TicksPerInch) +
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
   * @param power power from joystick, ex: -1.0 to 1.1
   */
  public final void manualPower(double power) {
    if ((this.getTicks() < 0 && power < 0) || (this.getTicks() > this.inchesToTicks(SlideHeight.MaxHeight) && power > 0)) return;
    this.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
    this.SlideMotor.setPower(power);
    this.Status = SlideStatus.ManualPower;
  }

  /**
   * Sets the target position of the motors
   *
   * @param height The target position as percentage
   */
  public final void setHeight(double height, double speed) {
    int ticks = this.inchesToTicks(height);

    if (ticks < 0 || ticks > this.inchesToTicks(SlideHeight.MaxHeight)) return;

    this.setTarget(ticks);
    this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    this.setPower(speed);

    this.Status = SlideStatus.MovingToTarget;
  }

  /**
   * Holds the slide at its current position
   */
  public final void holdHeight() {
    // this.Status = SlideStatus.Holding;

    // this.setTarget(this.getTicks());
    // this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // this.setPower(SlideSpeed.Hold);
  }

  /**
   * Updates the slide with controls
   * @param power The power to set the motors to, from -1.0 to 1.0 from the gamepad2 left stick y
   * @param up The up button on the dpad
   * @param left The left button on the dpad
   * @param down The down button on the dpad
   * @param right The right button on the dpad
   */
  public final void updateWithControls(
    double power,
    boolean up,
    boolean left,
    boolean down,
    boolean right
  ) {
    if (this.getTicks() == this.SlideMotor.getTargetPosition()) {
      this.Status = SlideStatus.Holding; // sets the status to holding if the slide is at its target
    }

    if (up) this.setHeight(SlideHeight.HighPole, this.Speed); // Slide set to high pole height if dpad up is pressed
    else if (left) this.setHeight(SlideHeight.MidPole, this.Speed); // Slide set to mid pole height if dpad left is pressed
    else if (down) this.setHeight(SlideHeight.LowPole, this.Speed); // Slide set to low pole height if dpad down is pressed
    else if (right) this.setHeight(SlideHeight.Ground, this.Speed); // Slide set to ground height if dpad right is pressed
    else if (power != 0) this.manualPower(power); // Slide set to power from gamepad2 left stick y if no dpad buttons are pressed
    // else if(this.Status == SlideStatus.Holding) this.holdHeight(); // Slide set to stop if no dpad buttons are pressed and gamepad2 left stick y is 0
    // else if(!this.SlideMotor.isBusy());

    if (this.getTicks() < 0) this.setPower(0);
    if (this.getInches() > SlideHeight.MaxHeight) this.setPower(0);

  }

  /**
   * Checks if slide has cleared the safety margin for the arm to move
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
}
