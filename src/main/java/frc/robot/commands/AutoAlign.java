// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveDriveWPI;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import static frc.robot.Constants.INCHES_TO_METERS;

/**
 * This command is a general command to handle specifically aligning to a given
 * tag.
 */
public class AutoAlign extends AutoCommand {

  // Declare local variables
  private SwerveDriveWPI swerve;
  private NetworkTableQuerier.TagCollection tags;

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
   * @param tags   the tag collection in network tables to use
   * @param tag    the tag ID we want to use
   * @param align  the alignment we want relative to the tag
   */
  public AutoAlign(SwerveDriveWPI swerve, NetworkTableQuerier.TagCollection tags, int tag, Alignment align) {
    super(Double.POSITIVE_INFINITY);

    // Set local variables
    this.swerve = swerve;
    this.tags = tags;
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
    // grab a lock to ensure indices don't change while we're doing this
    // we *probably* don't have to, but I'd rather just put this in now than try to
    // figure out why it's mysteriously breaking in a week
    tags.lock.readLock().lock();
    try {
      tags.tagIndex(tagId).ifPresent(idx -> {
        // start by setting our done flag to true, it'll be set to false if we aren't at
        // our target
        done = true;

        // compute distance and offset from the target
        double distance = tags.distances[idx] * INCHES_TO_METERS - align.distance;
        double offset = tags.offsets[idx] * INCHES_TO_METERS - align.offset;
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

        double rotError = tags.rotations[idx] - align.rotation;
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
      });
    } finally {
      tags.lock.readLock().unlock();
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
    return done;
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
}
