// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveWPI;
import static frc.robot.Constants.INCHES_TO_METERS;

import java.util.Optional;

/**
 * This command is a general command to handle specifically aligning to a given
 * tag.
 */
public abstract class AutoAlignBase extends AutoDrive {

  protected Alignment align;
  protected boolean foundTag;

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
    public double distTolerance = 0.05;
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
    super(swerve);
    this.align = align;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var pos = getTagPosition();
    foundTag = pos.isPresent();
    pos.ifPresent(tag -> {
      setFieldOriented(false);
      setDriveForward(Math.sqrt(tag.distance() * tag.distance() - tag.offset() * tag.offset()) * INCHES_TO_METERS - align.distance);
      setDriveRight(-tag.offset() * INCHES_TO_METERS + align.offset);
      setRotate(-tag.rotation() - align.rotation);
      super.initialize();
    });
  }

  @Override
  public boolean isFinished() {
    return !foundTag || super.isFinished();
  }

  /**
   * Get the position of the targeted tag
   * 
   * @return the tag position
   */
  protected abstract Optional<TagPosition> getTagPosition();
}
