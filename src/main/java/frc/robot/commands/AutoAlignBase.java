// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveDriveWPI;
import static frc.robot.Constants.INCHES_TO_METERS;

import java.time.Duration;
import java.time.Instant;
import java.util.Optional;

/**
 * This command is a general command to handle specifically aligning to a given
 * tag.
 */
public abstract class AutoAlignBase extends AutoCommand {

  // Declare local variables
  private SwerveDriveWPI swerve;

  private int tagId;
  private Alignment align;

  private PIDController distController;
  private PIDController rotController;

  private static final double dist_kP = 1.0;
  private static final double dist_kI = 0.0;
  private static final double dist_kD = 0.0;

  private static final double rot_kP = 1.0;
  private static final double rot_kI = 0.0;
  private static final double rot_kD = 0.0;

  private boolean done;
  private double distError;

  protected Instant lastSuccess;

  /**
   * How long we should go before stopping if we don't have a target
   */
  protected static final Duration maxTimeout = Duration.ofMillis(250);

  /**
   * Relative position of the targeted april tag.
   */
  protected static record TagPosition(double distance, double offset, double rotation) {
  }

  /**
   * Alignment that we want relative to a tag.
   */
  public static class Alignment {
    /**
     * Distance from the tag, in meters.
     */
    public double distance;
    /**
     * Offset is the length perpendicular to the distance, to the right.
     * This is measured in meters.
     */
    public double offset;
    /**
     * Rotation of the tag, in radians.
     */
    public double rotation;

    /**
     * Tolerance for our distance from the target position, in meters.
     */
    public double distTolerance = 0.01;
    /**
     * Tolerance for our rotational difference from the target position, in radians.
     */
    public double rotTolerance = 0.02;
  }

  /**
   * Creates a new command to automatically align to a given tag
   * 
   * @param swerve the swerve drive
   * @param align  the alignment we want relative to the tag
   */
  public AutoAlignBase(SwerveDriveWPI swerve, Alignment align) {
    super(Double.POSITIVE_INFINITY);

    // Set local variables
    this.swerve = swerve;
    this.align = align;

    distController = new PIDController(dist_kP, dist_kI, dist_kD);
    rotController = new PIDController(rot_kP, rot_kI, rot_kD);

    // Declare subsystem requirements
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    preExectute();
    try {
      getTagPosition().ifPresentOrElse(pos -> {
        // start by setting our done flag to true, it'll be set to false if we aren't at
        // our target
        done = true;

        // compute distance and offset from the target
        double distance = pos.distance * INCHES_TO_METERS - align.distance;
        double offset = pos.offset * INCHES_TO_METERS - align.offset;
        distError = Math.sqrt(distance * distance + offset * offset);

        // apply a tolerance deadband
        if (distError < align.distTolerance)
          distError = 0;
        else
          done = false;

        SmartDashboard.putNumber("Target Distance", distError);
        SmartDashboard.putNumber("Target Offset", offset);
        SmartDashboard.putNumber("Distance Error", distError);

        // calculate our speed from a PID controller, and angle just from the distance
        // and offset
        double speed = distController.calculate(distError);
        double angle = Math.atan2(distance, offset);

        SmartDashboard.putNumber("Drive Speed", speed);
        SmartDashboard.putNumber("Angle to Target", angle);

        double cos = Math.cos(angle);
        double sin = Math.sqrt(1 - cos * cos);

        double rotError = pos.rotation - align.rotation;
        // this corrects the rotation error to keep it in [-PI, PI]
        if (rotError > Math.PI)
          rotError -= 2 * Math.PI;
        else if (rotError < -Math.PI)
          rotError += 2 * Math.PI;

        // apply a rotation deadband from our tolerance
        if (Math.abs(rotError) < align.rotTolerance)
          rotError = 0;
        else
          done = false;
        SmartDashboard.putNumber("Rotation Error", rotError);

        double rotSpeed = rotController.calculate(rotError);
        SmartDashboard.putNumber("Rotation Speed", rotSpeed);
        swerve.driveRobotAuto(new ChassisSpeeds(speed * cos, speed * sin, rotSpeed)); // TODO: check sin/cos for angles

        // set success
        lastSuccess = Instant.now();
      }, () -> {
        if (lastSuccess == null || Duration.between(lastSuccess, Instant.now()).compareTo(maxTimeout) > 0)
          swerve.stopDrive();
      });
    } finally {
      postExecute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished() || done;
  }

  /**
   * Get the currently focused tag
   * 
   * @return the tag ID we're focused on
   */
  public int getTagId() {
    return tagId;
  }

  /**
   * Set a new focused tag
   * 
   * @param newId the new tag ID to focus
   */
  public void setTagId(int newId) {
    tagId = newId;
  }

  /**
   * Get the distance to our target position
   * 
   * @return the distance in meters
   */
  public double getDistance() {
    return distError;
  }

  /**
   * Get the position of the targeted tag
   * 
   * @return the tag position
   */
  protected abstract Optional<TagPosition> getTagPosition();

  /**
   * Code to be run at the start of the call to execute()
   */
  protected void preExectute() {
  }

  /**
   * COde to be run at the end of the call to execute()
   */
  protected void postExecute() {
  }
}
