package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Chassis class which contains all the methods for the chassis of the robot during autonomous
 */
public class AutoChassis {

  /**
   *
   * --- --- --- --- --- --- --- --- Important Chassis Variables --- --- --- --- --- --- --- ---
   *
   */

  /**
   * The wheels of the chassis
   */
  private Wheels Wheels;

  /**
   * The position of the chassis
   */
  private ChassisPosition Position = new ChassisPosition(0.0, 0.0, 0.0);

  /**
   * Creates a new chassis with 4 motors
   * @param hardwareMap The hardware map of the robot
   */
  public AutoChassis(HardwareMap hardwareMap) {
    this.Wheels =
      new Wheels(
        hardwareMap.get(DcMotor.class, "frontLeft"),
        hardwareMap.get(DcMotor.class, "frontRight"),
        hardwareMap.get(DcMotor.class, "backLeft"),
        hardwareMap.get(DcMotor.class, "backRight")
      );

    this.Wheels.FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    this.Wheels.BackLeft.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Wheel Class --- --- --- --- --- --- --- ---
   *
   */

  /**
   * The motors of the chassis
   */
  private class Wheels {

    private final DcMotor FrontLeft;
    private final DcMotor FrontRight;
    private final DcMotor BackLeft;
    private final DcMotor BackRight;

    /**
     * Constructor for the wheels of the chassis
     * @param frontLeft The front left motor
     * @param frontRight The front right motor
     * @param backLeft The back left motor
     * @param backRight The back right motor
     */
    public Wheels(
      DcMotor frontLeft,
      DcMotor frontRight,
      DcMotor backLeft,
      DcMotor backRight
    ) {
      this.FrontLeft = frontLeft;
      this.FrontRight = frontRight;
      this.BackLeft = backLeft;
      this.BackRight = backRight;
    }

    /**
     * Returns the 4 motors in an array
     * @return Array of motors
     */
    public DcMotor[] getMotors() {
      return new DcMotor[] {
        this.FrontLeft,
        this.FrontRight,
        this.BackLeft,
        this.BackRight,
      };
    }
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Autonomous Movement Methods --- --- --- --- --- --- --- ---
   *
   */

  /**
   * Moves the robot forward a certain distance in inches
   * @param distance The distance to move forward in inches
   * @return AutoChassis
   */
  public final AutoChassis forward(double distance) {
    this._moveDistance(distance, distance, distance, distance);

    return this;
  }

  /**
   * Moves the robot backward a certain distance in inches
   * @param distance The distance to move backward in inches
   * @return AutoChassis
   */
  public final AutoChassis backward(double distance) {
    this._moveDistance(-distance, -distance, -distance, -distance);

    return this;
  }

  /**
   * Moves the robot left a certain distance in inches
   * @param distance The distance to move left in inches
   * @return AutoChassis
   */
  public final AutoChassis strafeLeft(double distance) {
    this._moveDistance(-distance, distance, distance, -distance);

    return this;
  }

  /**
   * Moves the robot right a certain distance in inches
   * @param distance The distance to move right in inches
   * @return AutoChassis
   */
  public final AutoChassis strafeRight(double distance) {
    this._moveDistance(distance, -distance, -distance, distance);

    return this;
  }

  /**
   * Rotates the robot left a certain distance in inches
   * @param distance The distance to rotate left in inches
   * @return AutoChassis
   */
  public final AutoChassis turnLeft(double distance) {
    this.setSpeed(ChassisConstants.AngularVelocity);
    this._moveDistance(-distance, distance, -distance, distance);

    return this;
  }

  /**
   * Rotates the robot right a certain distance in inches
   * @param distance The distance to rotate right in inches
   * @return AutoChassis
   */
  public final AutoChassis turnRight(double distance) {
    this.setSpeed(ChassisConstants.AngularVelocity);
    this._moveDistance(distance, -distance, distance, -distance);

    return this;
  }

  /**
   * Moves the robot forward at a speed until the condition is met
   * @param speed The speed to move forward at
   * @param condition The condition to stop moving forward at
   * @return AutoChassis
   */
  public final AutoChassis forwardUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(speed, speed, speed, speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   * Moves the robot backward at a speed until the condition is met
   * @param speed The speed to move backward at
   * @param condition The condition to stop moving backward at
   * @return AutoChassis
   */
  public final AutoChassis backwardUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(-speed, -speed, -speed, -speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   * Moves the robot left at a speed until the condition is met
   * @param speed The speed to move left at
   * @param condition The condition to stop moving left at
   * @return AutoChassis
   */
  public final AutoChassis strafeLeftUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(-speed, speed, speed, -speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   * Moves the robot right at a speed until the condition is met
   * @param speed The speed to move right at
   * @param condition The condition to stop moving right at
   * @return AutoChassis
   */
  public final AutoChassis strafeRightUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(speed, -speed, -speed, speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   * Rotates the robot left at a speed until the condition is met
   * @param speed The speed to rotate left at
   * @param condition The condition to stop rotating left at
   * @return AutoChassis
   */
  public final AutoChassis turnLeftUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(-speed, speed, -speed, speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   * Rotates the robot right at a speed until the condition is met
   * @param speed The speed to rotate right at
   * @param condition The condition to stop rotating right at
   * @return AutoChassis
   */
  public final AutoChassis turnRightUntil(
    double speed,
    UntilCondition condition
  ) {
    this._setTargetPower(speed, -speed, speed, -speed);
    this.UntilCondition = condition;

    return this;
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Autonomous Movement Variables --- --- --- --- --- --- --- ---
   *
   */

  /**
   * The target power of each motor when running an auto movement
   */
  private static final class TargetPower {

    public double FrontLeft;
    public double FrontRight;
    public double BackLeft;
    public double BackRight;

    public void reset() {
      this.FrontLeft = 0.0;
      this.FrontRight = 0.0;
      this.BackLeft = 0.0;
      this.BackRight = 0.0;
    }

    public TargetPower() {
      reset();
    }
  }

  TargetPower TargetPower = new TargetPower();

  /**
   * The speed of the chassis when running an auto movement
   */
  private double RunSpeed = ChassisConstants.LinearVelocity;

  /**
   * Interface for background task
   */
  interface BackgroundTask {
    void call();
  }

  /**
   * Interface for Until Condition
   */
  interface UntilCondition {
    boolean call();
  }

  /**
   * The background task that will run when chassis is moving
   */
  private BackgroundTask BackgroundTask = null;

  /**
   * The condition that will be checked when chassis is moving
   */
  private UntilCondition UntilCondition = null;

  /**
   *
   * --- --- --- --- --- --- --- --- Autonomous Movement Util Methods --- --- --- --- --- --- --- ---
   *
   */

  /**
   * Sets the power of each motor based of this.RunSpeed
   */
  private final void _runMotors() {
    this._setPower(this.RunSpeed, this.RunSpeed, this.RunSpeed, this.RunSpeed);
  }

  /**
   * Sets the speed the chassis will move at when running a movement method
   * @param speed The speed to set the chassis to
   * @return AutoChassis
   */
  public final AutoChassis setSpeed(double speed) {
    this.RunSpeed = speed;

    return this;
  }

  /**
   * Moves the chassis based on the distances per wheel in inches
   * @param frontLeft The distance the front left wheel should move in inches
   * @param frontRight The distance the front right wheel should move in inches
   * @param backLeft The distance the back left wheel should move in inches
   * @param backRight The distance the back right wheel should move in inches
   */
  private final void _moveDistance(
    double frontLeft,
    double frontRight,
    double backLeft,
    double backRight
  ) {
    this._setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset the encoders

    this._setTargetPositions(
        (int) (frontLeft * ChassisConstants.TicksPerInch),
        (int) (frontRight * ChassisConstants.TicksPerInch),
        (int) (backLeft * ChassisConstants.TicksPerInch),
        (int) (backRight * ChassisConstants.TicksPerInch)
      ); //set the target positions of the motors based on the distance

    this._setMode(DcMotor.RunMode.RUN_TO_POSITION); //set the motors to run to position
  }

  /**
   * Sets the task in the background that will run when chassis is moving
   * @param task The task to run in the background, ex () -> { ... }
   * @return AutoChassis
   */
  public final AutoChassis runBackgroundTask(BackgroundTask task) {
    this.BackgroundTask = task;

    return this;
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Chassis Util Methods --- --- --- --- --- --- --- ---
   *
   */

  /**
   * Sets the mode of all the motors
   * @param mode The mode to set the motors to (RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION)
   */
  private final void _setMode(DcMotor.RunMode mode) {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setMode(mode);
    }
  }

  /**
   * Sets the target position of each motor
   * @param frontLeft The target position of the front left motor in ticks
   * @param frontRight The target position of the front right motor in ticks
   * @param backLeft The target position of the back left motor in ticks
   * @param backRight The target position of the back right motor in ticks
   */
  private final void _setTargetPositions(
    int frontLeft,
    int frontRight,
    int backLeft,
    int backRight
  ) {
    this.Wheels.FrontLeft.setTargetPosition(frontLeft);
    this.Wheels.FrontRight.setTargetPosition(frontRight);
    this.Wheels.BackLeft.setTargetPosition(backLeft);
    this.Wheels.BackRight.setTargetPosition(backRight);
  }

  /**
   * Sets the target power of each motor
   * @param frontLeft The target power of the front left motor
   * @param frontRight The target power of the front right motor
   * @param backLeft The target power of the back left motor
   * @param backRight The target power of the back right motor
   */
  private final void _setTargetPower(
    double frontLeft,
    double frontRight,
    double backLeft,
    double backRight
  ) {
    this.TargetPower.FrontLeft = frontLeft;
    this.TargetPower.FrontRight = frontRight;
    this.TargetPower.BackLeft = backLeft;
    this.TargetPower.BackRight = backRight;
  }

  /**
   * Sets the power of each motor
   * @param frontLeft The power of the front left motor
   * @param frontRight The power of the front right motor
   * @param backLeft The power of the back left motor
   * @param backRight The power of the back right motor
   */
  private final void _setPower(
    double frontLeft,
    double frontRight,
    double backLeft,
    double backRight
  ) {
    this.Wheels.FrontLeft.setPower(frontLeft);
    this.Wheels.FrontRight.setPower(frontRight);
    this.Wheels.BackLeft.setPower(backLeft);
    this.Wheels.BackRight.setPower(backRight);
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Autonomous Localization Util Methods --- --- --- --- --- --- --- ---
   */

  /**
   *
   * --- --- --- --- --- --- --- --- Autonomous Status Util Methods --- --- --- --- --- --- --- ---
   */

  /**
   * Resets the chassis after it has finished moving
   */
  private final void _reset() {
    this.RunSpeed = ChassisConstants.LinearVelocity;
    this.TargetPower.reset();
    this.BackgroundTask = null;
    this.UntilCondition = null;
  }

  /**
   * Stops the chassis
   */
  private final void _stop() {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setPower(0);
    }
  }

  /**
   * Runs the automated movement of the chassis
   */
  public final void run() {
    this._runMotors();

    this._idle();

    this._stop();

    this._reset();
  }

  /**
   * Runs the automated movement of the chassis until the condition is met
   */
  public final void runUntil() {
    this._setPower(
        this.TargetPower.FrontLeft,
        this.TargetPower.FrontRight,
        this.TargetPower.BackLeft,
        this.TargetPower.BackRight
      ); // Set the power of the motors to the target power

    while (!this.UntilCondition.call()) { //while loop until condition is met
      this.update();
      if (this.BackgroundTask != null) {
        this.BackgroundTask.call(); //calls the background task if there is one
      }
    }
    this._stop(); //stops the chassis

    this._reset(); //resets the chassis for next movement
  }

  /**
   * Waits for the motors to finish running, also runs the background task and update method
   */
  private final void _idle() {
    while (
      this.Wheels.FrontLeft.isBusy() &&
      this.Wheels.FrontRight.isBusy() &&
      this.Wheels.BackLeft.isBusy() &&
      this.Wheels.BackRight.isBusy()
    ) {
      this.update();
      if (this.BackgroundTask != null) {
        this.BackgroundTask.call();
      }
    }
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Chassis Public Util Methods --- --- --- --- --- --- --- ---
   *
   */

  public final ChassisPosition getPosition() {
    return this.Position;
  }

  /**
   *
   * --- --- --- --- --- --- --- --- Chassis Update Methods --- --- --- --- --- --- --- ---
   *
   */

  /**
   * Updates the position of the chassis and other background tasks
   */
  public final void update() {
    // Field.drawRobot(this.Position);
  }

}
