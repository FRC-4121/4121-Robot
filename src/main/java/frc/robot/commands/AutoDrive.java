// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveWPI;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoDrive extends Command {

  /** Creates a new AutoDrive. */
  protected final SwerveDriveWPI drive;

  protected static final class Gains {
    public static final double drive_kP = 1.0;
    public static final double drive_kI = 0.0;
    public static final double drive_kD = 0.0;

    public static final double rot_kP = 0.6;
    public static final double rot_kI = 0.1;
    public static final double rot_kD = 0.0;
  }

  protected PIDController driveControl;
  protected PIDController rotControl;

  protected boolean fieldOriented;

  protected double linearSpeed = 2.0;
  protected double angularSpeed = 0.5;

  private double dist;
  private double targetGyro;

  protected double dx;
  protected double dy;
  protected double dr;
  
  public AutoDrive(SwerveDriveWPI drive) {
    this.drive = drive;
    driveControl = new PIDController(Gains.drive_kP, Gains.drive_kI, Gains.drive_kD);
    rotControl = new PIDController(Gains.rot_kP, Gains.rot_kI, Gains.rot_kD);
  }

  public AutoDrive setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
    return this;
  }
  public AutoDrive setDx(double dx) {
    this.dx = dx;
    return this;
  }
  public AutoDrive setDy(double dy) {
    this.dy = dy;
    return this;
  }
  public AutoDrive setDr(double dr) {
    this.dr = dr;
    return this;
  }

  @Override
  public void initialize() {
    drive.resetDistance();
    dist = Math.sqrt(dx * dx + dy * dy);
    double gyroRadians = Math.toRadians(drive.getGyroAngleField());
    targetGyro = (gyroRadians + dr) % (2 * Math.PI);
    SmartDashboard.putNumber("Auto dX", dx);
    SmartDashboard.putNumber("Auto dY", dy);
    SmartDashboard.putNumber("Auto dR", dr);
    SmartDashboard.putNumber("Auto Target Gyro", targetGyro);
  }

  @Override
  public void execute() {
    double gyroRadians = Math.toRadians(drive.getGyroAngleField());
    double rotErr = (gyroRadians - targetGyro) % (2 * Math.PI);
    if (rotErr > Math.PI) rotErr -= Math.PI * 2;
    else if (rotErr < -Math.PI) rotErr += Math.PI * 2;
    SmartDashboard.putNumber("Auto Rot Error", rotErr);
    double rightX = rotControl.calculate(-rotErr) * angularSpeed;
    double distErr = distanceToTarget();
    SmartDashboard.putNumber("Auto Drive Dist Error", distErr);
    double scale = driveControl.calculate(-distErr);
    SmartDashboard.putNumber("Auto Drive PID Scale", scale);
    double leftX = dx / dist * scale * linearSpeed;
    double leftY = dy / dist * scale * linearSpeed;
    var speeds = new ChassisSpeeds(leftX, leftY, rightX);
    if (fieldOriented) drive.driveFieldRelative(speeds);
    else drive.driveRobot(speeds);
  }

  @Override
  public boolean isFinished() {
    return drive.calculateDriveDistance() >= dist;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopDrive();
  }

  protected double distanceToTarget() {
    return dist - Math.abs(drive.calculateDriveDistance());
  }
}
