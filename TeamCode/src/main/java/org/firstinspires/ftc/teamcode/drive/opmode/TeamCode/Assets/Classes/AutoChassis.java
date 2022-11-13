package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Chassis class which contains all the methods for the chassis of the robot during autonomous
 */
public class AutoChassis {

  /**
   * The motors of the chassis
   */
  private class Wheels {

    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;

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

  private Wheels Wheels;

  public AutoChassis(
    HardwareMap hardwareMap
  ) {
    this.Wheels = new Wheels(
        hardwareMap.get(DcMotor.class, "frontLeft"),
        hardwareMap.get(DcMotor.class, "frontRight"),
        hardwareMap.get(DcMotor.class, "backLeft"),
        hardwareMap.get(DcMotor.class, "backRight")
    );

    this.Wheels.FrontLeft.setDirection(DcMotor.Direction.REVERSE);
    this.Wheels.BackLeft.setDirection(DcMotor.Direction.REVERSE);
  }

  public final void setMode(DcMotor.RunMode mode) {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setMode(mode);
    }
  }

  public final void update() {

  }

  public final void stop() {
    for (DcMotor motor : this.Wheels.getMotors()) {
      motor.setPower(0);
    }
  }

  public class Move {
    public final void Forward(double distance) {

    }

    public final void Backward(double distance) {

    }

    public final void Left(double distance) {

    }

    public final void Right(double distance) {

    }

    public final void TurnLeft(double distance) {

    }
    
    public final void TurnRight(double distance) {

    }

    public final void StrafeLeft(double distance) {

    }

    public final void StrafeRight(double distance) {

    }
  }
}
