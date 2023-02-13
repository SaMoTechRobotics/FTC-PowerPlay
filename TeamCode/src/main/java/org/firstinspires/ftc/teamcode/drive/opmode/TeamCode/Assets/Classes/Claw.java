package org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Classes;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Arm.ArmRotation;
import org.firstinspires.ftc.teamcode.drive.opmode.TeamCode.Assets.Constants.Claw.ClawPosition;

/**
 * Claw class which contains all the methods for the claw of the robot
 */
public class Claw {

    private final Servo ClawServo;

    private final Servo PoleBraceServo;
    private final DistanceSensor ClawDistanceSensor;

    private boolean open = false;
    private boolean detectedCone = false;

    private double OpenAmount = ClawPosition.PickupOpen;


    /**
     * Creates a new claw with 1 servo
     *
     * @param ClawServo The servo that will be used for the claw
     */
    public Claw(Servo ClawServo, DistanceSensor ClawDistanceSensor, Servo PoleBraceServo) {
        this.ClawServo = ClawServo;
        this.ClawDistanceSensor = ClawDistanceSensor;
        this.PoleBraceServo = PoleBraceServo;
    }

//    public Claw(Servo ClawServo, DistanceSensor ClawDistanceSensor) {
//        this.ClawServo = ClawServo;
//        this.ClawDistanceSensor = ClawDistanceSensor;
//    }

    public final double getPosition() {
        return this.ClawServo.getPosition();
    }

    /**
     * Sets the position of the servo
     *
     * @param position The position to set the servo to as percentage 0-100
     */
    public final void setPosition(double position) {
        this.ClawServo.setPosition(
                this.percentToPosition(position) // Converts the position to a percentage
        );
    }

    public final void setOpenAmount(double openPosition) {
        this.OpenAmount = openPosition;
    }

    public final boolean isOpen() {
        return this.open;
    }

    /**
     * Opens the claw
     */
    public final void open() {
        this.open = true;
//        this.setPosition(ClawPosition.Open);
        this.setPosition(this.OpenAmount);
    }

    /**
     * Closes the claw
     */
    public final void close() {
        this.open = false;
        this.setPosition(ClawPosition.Close);
    }

    /**
     * Toggles the claw
     *
     * @param open The open position of the claw
     */
    public final void toggleOpen(boolean open) {
        this.open = !this.open;
        if (this.open) {
            this.open();
        } else {
            this.close();
        }
    }

    public final double getSensorDistance() {
        return this.ClawDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public final boolean detectedCone() {
        return this.getSensorDistance() < ClawPosition.ConeDistance;
    }

    public final boolean justDetectedCone() {
        if (this.detectedCone() && !this.detectedCone) {
            this.detectedCone = true;
            return true;
        } else if (!this.detectedCone()) {
            this.detectedCone = false;
        }
        return false;
    }


    public final void setPoleBracePosition(double position) {
        this.PoleBraceServo.setPosition(position);
    }

    public final void raisePoleBrace() {
        this.PoleBraceServo.setPosition(ClawPosition.PoleBraceUp);
    }

    public final void lowerPoleBrace(double armRotation, ClawPosition.PoleBraceAlignDirection direction) {
        if (direction == ClawPosition.PoleBraceAlignDirection.Backward) {
            if (!ClawPosition.ReversePoleBrace ? armRotation > ArmRotation.Center : armRotation < ArmRotation.Center) {
                this.setPoleBracePosition(ClawPosition.PoleBraceDownLeft);
            } else {
                this.setPoleBracePosition(ClawPosition.PoleBraceDownRight);
            }
        } else {
            if (ClawPosition.ReversePoleBrace ? armRotation > ArmRotation.Center : armRotation < ArmRotation.Center) {
                this.setPoleBracePosition(ClawPosition.PoleBraceDownLeft);
            } else {
                this.setPoleBracePosition(ClawPosition.PoleBraceDownRight);
            }
        }
    }

    /**
     * Converts a percentage to a servo position
     *
     * @param percent The percentage to convert as an int
     * @return The servo position as a double
     */
    private final double percentToPosition(double percent) {
        return percent / 100;
    }

}
