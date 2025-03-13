// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveWPI;
import edu.wpi.first.math.controller.*;

public class AutoDrive extends Command {

  /** Creates a new AutoDrive. */
  protected final SwerveDriveWPI drive;

  protected static final class Gains {
    public static final double drive_kP = 0.2;
    public static final double drive_kI = 0.0;
    public static final double drive_kD = 0.0;

    public static final double rot_kP = 1.0;
    public static final double rot_kI = 0.0;
    public static final double rot_kD = 0.0;
  }

  protected PIDController driveControl;
  protected PIDController rotControl;

  // we use field-oriented drive; if we aren't, rotate our inputs to be used for it
  protected boolean fieldOriented;
  // "forward" amount for robot-oriented, "away" amount for field-oriented
  protected double driveForward;
  // "right" amount for both orientations
  protected double driveRight;
  // rotation amount in rotations
  protected double rotate;

  private double dist;
  private double dx;
  private double dy;
  private double dr;
  
  public AutoDrive(SwerveDriveWPI drive) {
    this.drive = drive;
    driveControl = new PIDController(Gains.drive_kP, Gains.drive_kI, Gains.drive_kD);
    rotControl = new PIDController(Gains.rot_kP, Gains.rot_kI, Gains.rot_kD);
  }

  public AutoDrive setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
    return this;
  }
  public AutoDrive setDriveForward(double driveForward) {
    this.driveForward = driveForward;
    return this;
  }
  public AutoDrive setDriveRight(double driveRight) {
    this.driveRight = driveRight;
    return this;
  }
  public AutoDrive setRotate(double rotate) {
    this.rotate = rotate;
    return this;
  }

  @Override
  public void initialize() {
    drive.resetDistance();
    if (fieldOriented) {
      dr = rotate;
      dx = driveRight;
      dy = driveForward;
    } else {
      double gyro = -Math.toRadians(drive.getGyroAngleField() + 90);
      double cos = Math.cos(gyro);
      double sin = Math.sqrt(1 - cos * cos);
      // dr = rotate - gyro;
      dr = rotate;
      dx = driveRight * cos + driveForward * sin;
      dy = driveForward * cos - driveRight * sin;
      dist = Math.sqrt(dx * dx + dy * dy);
    }
    SmartDashboard.putNumber("Auto dX", dx);
    SmartDashboard.putNumber("Auto dY", dy);
    SmartDashboard.putNumber("Auto dR", dr);
  }

  @Override
  public void execute() {
    double rotErr = Math.toRadians(drive.getGyroAngleField()) - dr;
    if (rotErr > Math.PI) rotErr -= Math.PI * 2;
    else if (rotErr < -Math.PI) rotErr += Math.PI * 2;
    double rightX = rotControl.calculate(rotErr);
    double distErr = dist - Math.abs(drive.calculateDriveDistance());
    SmartDashboard.putNumber("Auto Drive Dist Error", distErr);
    double scale = driveControl.calculate(distErr);
    double leftX = dy * scale;
    double leftY = dx * scale;
    drive.driveFieldRelative(leftX, leftY, 0);
  }

  @Override
  public boolean isFinished() {
    return drive.calculateDriveDistance() >= dist;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopDrive();
  }
}
