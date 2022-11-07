package org.firstinspires.ftc.teamcode;
// package org.firstinspires.ftc.teamcode.drive.opmode.teamcode.assets;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Arm class which contains all the methods for the arm of the robot
 */
public class Arm {

  private Servo ArmServo;

  /**
   * Creates a new arm with 1 servo
   *
   * @param ArmServo The servo that will be used for the arm
   */
  public Arm(Servo ArmServo) {
    this.ArmServo = ArmServo;

  }

  /**
   * Sets the position of the servo
   *
   * @param position The position to set the servo to as rotation in degrees
   */
  public final void setRotation(double rotation) {
    this.ArmServo.setPosition(
        this.rotationToPosition(rotation) // Converts the position to a percentage
    );
  }

  /**
   * Rotates the arm
   * @param speed The speed to rotate the arm at
   */
  public final void addRotation(double speed) {
    // this.ArmServo.setPower(speed);
  }

  /**
   * Updates the arm with controls
   * @param speed The speed to rotate the arm at from gamepad2 left stick y
   * @param left The left bumper of gamepad2
   * @param right The right bumper of gamepad2
   * @param center The y button of gamepad2
   */
  public final void updateWithControls(
    double speed,
    boolean left,
    boolean right,
    boolean center
  ) {
    if (center) {
      this.setRotation(ArmRotation.Center); // Sets the rotation to the middle
    } else if (left) {
      this.setRotation(ArmRotation.Left); // Sets the rotation to the left
    } else if (right) {
      this.setRotation(ArmRotation.Right); // Sets the rotation to the right
    } else {
      // this.addRotation(speed); // Adds the speed to the rotation
    }
  }

  /**
   * Converts a rotation to a servo position
   *
   * @param percent The rotation to convert to servo position
   * @return The servo position from rotation
   */
  private final double rotationToPosition(double rotation) {
    return rotation;
  }
}
