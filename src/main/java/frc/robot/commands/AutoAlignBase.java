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
  protected boolean inRange;
  protected ToMove toMove;

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
  }

  /**
   * 
   * How we want this command to move the robot
   * 
   */
  public static class ToMove {
    public boolean rotate;
    public boolean drive;

    public static final ToMove All = new ToMove() {
      {
        rotate = true;
        drive = true;
      }
    };
    public static final ToMove RotateOnly = new ToMove() {
      {
        rotate = true;
        drive = false;
      }
    };
    public static final ToMove DriveOnly = new ToMove() {
      {
        rotate = false;
        drive = true;
      }
    };
  }

  /**
   * Creates a new command to automatically align to a given tag
   * 
   * @param swerve the swerve drive
   * @param align  the alignment we want relative to the tag
   */
  public AutoAlignBase(SwerveDriveWPI swerve, Alignment align, ToMove toMove) {
    super(swerve);
    this.align = align;
    this.toMove = toMove;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var pos = getTagPosition();
    foundTag = pos.isPresent();
    inRange = true;
    pos.ifPresent(tag -> {
      setFieldOriented(false);
      if (toMove.drive) {
        setDx(distanceToTarget());
        setDy(tag.offset() * INCHES_TO_METERS - align.offset);
      } else {
        setDx(0);
        setDy(0);
      }
      if (toMove.rotate) {
        setDr(tag.rotation() - align.rotation);
      } else {
        setDr(0);
      }
      super.initialize();
    });
  }

  @Override
  public boolean isFinished() {
    return !foundTag || !inRange || ((!toMove.drive || isDriveFinished()) && (!toMove.rotate || isRotFinished()));
  }

  @Override
  protected double distanceToTarget() {
    if (!toMove.drive) return 0;
    double l1 = drive.getLeftFrontLaser();
    double l2 = drive.getRightFrontLaser();
    if (l1 < 0) {
      if (l2 < 0) {
        inRange = false;
        return 0;
      } else {
        inRange = true;
        return Math.max(l2 - align.distance, 0);
      }
    } else if (l2 < 0) {
      inRange = true;
      return Math.max(l1 - align.distance, 0);
    } else {
      inRange = true;
      return Math.max(Math.min(l1, l2) - align.distance, 0);
    }
  }

  /**
   * Get the position of the targeted tag
   * 
   * @return the tag position
   */
  protected abstract Optional<TagPosition> getTagPosition();
}
