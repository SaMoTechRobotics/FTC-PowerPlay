package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Slide class which contains all the methods for the slide of the robot
 */
public class Slide {

  private DcMotor SlideMotor;

  private double Speed = SlideSpeed.Max;

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
  }


  public final int getTicks() {
    return this.SlideMotor.getCurrentPosition();
  }

  /**
   * Sets the power of motors
   *
   * @param power The power to set the motors to, from -1.0 to 1.0
   */
  public final void setPower(double power) {
    this.SlideMotor.setPower(power);
  }

  /**
   * Sets the mode of the motors
   *
   * @param mode The mode to set the motors to, from DcMotor.RunMode enum
   */
  public final void setMode(DcMotor.RunMode mode) {
    this.SlideMotor.setMode(mode);
  }

  /**
   * Sets the target position of the motors
   *
   * @param target The target position in ticks
   */
  public final void setTarget(int target) {
    this.SlideMotor.setTargetPosition(target);
  }

  /**
   * Sets the target position of the motors
   *
   * @param height The target position as percentage
   */
  public final void setHeight(double height, double speed) {
    int ticks = this.percentToTicks(height);

    this.setTarget(ticks);
    this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    this.setPower(speed);

    while (this.SlideMotor.isBusy()) {
      // Wait for slide to reach target
    }

    this.setPower(SlideSpeed.Stop);
    this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    if (up) {
      this.setHeight(SlideHeight.HighPole, this.Speed); // Slide set to high pole height if dpad up is pressed
    } else if (left) {
      this.setHeight(SlideHeight.MidPole, this.Speed); // Slide set to mid pole height if dpad left is pressed
    } else if (down) {
      this.setHeight(SlideHeight.LowPole, this.Speed); // Slide set to low pole height if dpad down is pressed
    } else if (right) {
      this.setHeight(SlideHeight.Ground, this.Speed); // Slide set to ground height if dpad right is pressed
    } else if (power != 0) {
      this.setPower(power); // Slide set to power if left stick y is not 0
    } else {
      this.setPower(SlideSpeed.Stop); // Slide set to 0 if no buttons are pressed and left stick y is 0
    }
  }

  /**
   * Converts a percentage to ticks
   *
   * @param height The target position as percentage
   */
  private final int percentToTicks(double percent) {
    return (int) (percent / 100 * SlideHeight.MaxTicks);
  }
}
