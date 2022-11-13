package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Chassis class which contains all the methods for the chassis of the robot during autonomous
 */
public class AutoChassis {

  /**
   * Interface for background task
   */
  interface Callback {
    void call();
  }

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

  /* All Variables for AutoChassis Class */

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
   * @param hardwareMap
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

  /* Util Methods */

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
   * Updates the position of the chassis and other background tasks
   */
  public final void update() {}

  /**
   * Stops the chassis, emergencies only
   */
  public final void stop() {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setPower(0);
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
   * Sets the power of each motor based of this.RunSpeed
   */
  private final void _runMotors() {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setPower(this.RunSpeed);
    }
  }

  /* Other Methods */

  public final ChassisPosition getPosition() {
    return this.Position;
  }

  /* Auto Movement Variables */

  /**
   * The speed of the chassis when running an auto movement
   */
  private double RunSpeed = ChassisSpeed.MidDrive;

  /**
   * The background task that will run when chassis is moving
   */
  private Callback BackgroundTask = null;

  /* All Movement Methods */

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
   * Resets the chassis after it has finished moving
   */
  private final void _reset() {
    this.RunSpeed = ChassisSpeed.MidDrive;
    this.BackgroundTask = null;
  }

  public final AutoChassis runBackgroundTask(Callback task) {
    this.BackgroundTask = task;

    return this;
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
   * Runs the automated movement of the chassis
   */
  public final void run() {
    this._runMotors();

    this._idle();

    this._reset();
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
    this._setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    this._setTargetPositions(
        (int) (frontLeft * ChassisConstants.TicksPerInch),
        (int) (frontRight * ChassisConstants.TicksPerInch),
        (int) (backLeft * ChassisConstants.TicksPerInch),
        (int) (backRight * ChassisConstants.TicksPerInch)
      );
    this._setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

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
    this._moveDistance(-distance, distance, -distance, distance);

    return this;
  }

  /**
   * Rotates the robot right a certain distance in inches
   * @param distance The distance to rotate right in inches
   * @return AutoChassis
   */
  public final AutoChassis turnRight(double distance) {
    this._moveDistance(distance, -distance, distance, -distance);

    return this;
  }
}
