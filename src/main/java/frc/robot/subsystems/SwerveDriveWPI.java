// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Optional;
import java.util.stream.Collectors;
import java.math.*;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.Mutables;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ControlConstants.*;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Define a SwerveDrive object
 */
public class SwerveDriveWPI extends SubsystemBase {

  // Declare CAN IDs for swerve wheels
  private final int leftFrontDriveId = 1;
  private final int leftFrontAngleId = 2;
  private final int leftFrontCoderId = 3;
  private final int leftFrontLaserId = 13;

  private final int rightFrontDriveId = 4;
  private final int rightFrontAngleId = 5;
  private final int rightFrontCoderId = 6;
  private final int rightFrontLaserId = 14;

  private final int leftBackDriveId = 7;
  private final int leftBackAngleId = 8;
  private final int leftBackCoderId = 9;
  private final int leftBackLaserId = 15;

  private final int rightBackDriveId = 10;
  private final int rightBackAngleId = 11;
  private final int rightBackCoderId = 12;
  private final int rightBackLaserId = 16;

  // Declare swerve modules
  private SwerveWheel2 leftFront;
  private SwerveWheel2 leftBack;
  private SwerveWheel2 rightFront;
  private SwerveWheel2 rightBack;

  // Declare swerve kinematics and odometry objects
  private SwerveDriveKinematics kinematics;
  private SwerveModuleState frontLeftState;
  private SwerveModuleState frontRightState;
  private SwerveModuleState backLeftState;
  private SwerveModuleState backRightState;
  private Translation2d leftFrontTranslation;
  private Translation2d leftBackTranslation;
  private Translation2d rightFrontTranslation;
  private Translation2d rightBackTranslation;
  private ChassisSpeeds speeds;
  private SwerveDriveOdometry odometry;

  // Declare swerve module inputs
  private double frontLeftAngle;
  private double backLeftAngle;
  private double frontRightAngle;
  private double backRightAngle;

  // Declare NavX gyro Objects
  private AHRS gyro;
  private ADXRS450_Gyro fallbackGyro;

  // Declare misc variables
  private double joystickDeadband;

  // Declare speed variables
  private double vxMetersPerSecond;
  private double vyMetersPerSecond;

  // Declare distance calculation variables
  private double LeftFrontStartingEncoder;
  private double LeftRearStartingEncoder;
  private double RightFrontStartingEncoder;
  private double RighRearStartingEncoder;

  // Declare collision detection variables;
  private double lastLinearAccelX;
  private double lastLinearAccelY;

  // Declare PID controller
  private PIDController wpiPIDController;

  // Declare 2d Field
  private Field2d field;

  private static final boolean disableNavx = false;

  /**
   * 
   * Creates a new SwerveDrive
   * 
   */
  public SwerveDriveWPI() {

    // Initialize new swerve modules
    leftFront = new SwerveWheel2("LF", leftFrontDriveId, leftFrontAngleId, leftFrontCoderId, leftFrontLaserId);
    leftBack = new SwerveWheel2("LB", leftBackDriveId, leftBackAngleId, leftBackCoderId, leftBackLaserId);
    rightFront = new SwerveWheel2("RF", rightFrontDriveId, rightFrontAngleId, rightFrontCoderId, rightFrontLaserId);
    rightBack = new SwerveWheel2("RB", rightBackDriveId, rightBackAngleId, rightBackCoderId, rightBackLaserId);

    // Initialize swerve kinematics objects
    // 2025 robot chassis is 30" x 30"
    leftFrontTranslation = new Translation2d(0.311, 0.311);// X-0.297, Y-0.288
    leftBackTranslation = new Translation2d(-0.311, 0.311);// 0.229,0.292 for last years bot
    rightFrontTranslation = new Translation2d(0.311, -0.311);
    rightBackTranslation = new Translation2d(-0.311, -0.311);
    kinematics = new SwerveDriveKinematics(leftFrontTranslation, rightFrontTranslation, leftBackTranslation,
        rightBackTranslation);

    // Initialize NavX gyro
    gyro = null;
    try {
      gyro = new AHRS(NavXComType.kMXP_SPI);
    } catch (Exception ex) {
      DriverStation.reportError("Unable to connect to NavX: " + ex.toString(), false);
      System.out.println("Unable to connect to NavX: " + ex.toString());
    }
    fallbackGyro = new ADXRS450_Gyro();

    // gyro.calibrate();
    if (gyro.isConnected()) {
      gyro.reset();
      gyro.resetDisplacement();
    }

    // Initialize misc variables
    joystickDeadband = 0.05;
    lastLinearAccelX = 0.0;
    lastLinearAccelY = 0.0;

    // Create PID controller
    wpiPIDController = new PIDController(kAnglePIDkp, kAnglePIDki, kAnglePIDkd);
    wpiPIDController.setTolerance(0.1, 5);

    // Initialize swerve odometry object
    odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getModulePositions());

    // Initialize 2d field
    field = new Field2d();

    // Configure PathPlanner AutoBuilder
    try {

      RobotConfig ppConfig = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelativePP,
          new PPHolonomicDriveController(
              translationConstants,
              rotationConstants),
          ppConfig,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            // return alliance.get() == DriverStation.Alliance.Red;
            // }
            return false;

          },
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    // Show field data on SmartDashboard
    SmartDashboard.putData("Field", field);

  }

  /**
   * 
   * Things to check/do every robot cycle
   * 
   */
  @Override
  public void periodic() {

    // Zero the gyro on driver command
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro == 1) {
      SmartDashboard.putNumber("Zero Gyro", 0);
      // gyro.calibrate();
      zeroGyro();
    }

    // Update robot odometry
    odometry.update(getGyroRotation2d(), getModulePositions());

    SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose Yaw", odometry.getPoseMeters().getRotation().getRadians());

    SmartDashboard.putString("Pose", getPose().toString());

    // Update field position
    field.setRobotPose(getPose());

    double l1 = getLeftFrontLaser();
    double l2 = getRightFrontLaser();
    double l3 = getLeftBackLaser();
    double l4 = getRightBackLaser();
    SmartDashboard.putBoolean("Auto Align in Range", l1 >= 0 & l2 >= 0);

    SmartDashboard.putBoolean("Against Front",
        (l1 >= 0 && l1 <= 0.14 || l2 >= 0 && l2 <= 0.14) && Math.abs(l1 - l2) < 0.2);
    SmartDashboard.putBoolean("Against Rear",
        (l3 >= 0 && l3 <= 0.14 || l4 >= 0 && l4 <= 0.14) && Math.abs(l3 - l4) < 0.2);
  }

  public boolean againstFront() {
    double l1 = getLeftFrontLaser();
    double l2 = getRightFrontLaser();
    return (l1 >= 0 && l1 <= 0.14 || l2 >= 0 && l2 <= 0.14);
  }

  public boolean againstBack() {
    double l1 = getLeftBackLaser();
    double l2 = getRightBackLaser();
    return (l1 >= 0 && l1 <= 0.14 || l2 >= 0 && l2 <= 0.14) && Math.abs(l1 - l2) < 0.5;
  }

  /**
   * 
   * Drive the robot relative to robot coordinate system
   * 
   * @param robotRelativeSpeeds A set of chassis speeds
   * 
   */
  public void driveRobotRelativePP(ChassisSpeeds robotRelativeSpeeds) {
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
    // 0.02);

    driveRobotAuto(robotRelativeSpeeds);
  }

  /**
   * 
   * Drive the robot from joystick inputs relative to robot coordinate system
   * 
   * @param leftX  Left joystick X direction input
   * @param leftY  Left joystick Y direction input
   * @param rightX Right joystick X direction input
   * 
   */
  public void driveRobotRelative(double leftX, double leftY, double rightX) {

    // Convert joystick positions to linear speeds in meters/second
    vxMetersPerSecond = -(leftY * LinearSpeed);
    vyMetersPerSecond = -(leftX * LinearSpeed);

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < kJoystickTolerance) {
      double yawRate = getGyroYawRate();
      double pidOutput = wpiPIDController.calculate(yawRate, 0.0);
      if (!Mutables.isSlowMode) {
        omegaRadiansPerSecond = RotationalSpeed * pidOutput;
      } else {
        omegaRadiansPerSecond = SlowRadiansPerSecond * pidOutput;
      }
      omegaRadiansPerSecond = 0.0;
    } else {
      if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband) {
        omegaRadiansPerSecond = RotationalSpeedFast * rightX;
      } else {
        omegaRadiansPerSecond = RotationalSpeed * rightX;
      }
    }

    // Convert inputs to chassis speeds
    ChassisSpeeds relativeSpeeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

    // Run the swerve modules based on current status
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband
        && Math.abs(rightX) < joystickDeadband) {

      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();

    } else {

      if (!Mutables.isParked) {

        driveRobot(relativeSpeeds);

      }

    }

  }

  /**
   * 
   * Drive the robot relative to a field coordinate system
   * 
   * @param fieldRelativeSpeeds A set of chassis speeds
   * 
   */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    // Convert inputs to chassis speeds
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
        Rotation2d.fromDegrees(toWPIAngle(getGyroAngleField())));
    driveRobot(fieldSpeeds);
  }

  /**
   * 
   * Drive the robot from joystick inputs relative to a field coordinate system
   * 
   * @param leftX  Left joystick X direction input
   * @param leftY  Left joystick Y direction input
   * @param rightX Right joystick X direction input
   * 
   */
  public void driveFieldRelative(double leftX, double leftY, double rightX) {

    // Convert joystick positions to linear speeds in meters/second
    vxMetersPerSecond = -(leftY * LinearSpeed);
    vyMetersPerSecond = -(leftX * LinearSpeed);
    SmartDashboard.putNumber("vX Speed", vxMetersPerSecond);
    SmartDashboard.putNumber("vY Speed", vyMetersPerSecond);

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < kJoystickTolerance) {

      double yawRate = getGyroYawRate();
      double pidOutput = wpiPIDController.calculate(yawRate, 0.0);
      if (!Mutables.isSlowMode) {
        omegaRadiansPerSecond = RotationalSpeed * pidOutput;
      } else {
        omegaRadiansPerSecond = SlowRadiansPerSecond * pidOutput;
      }
      omegaRadiansPerSecond = 0.0;

    } else {

      if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband) {
        omegaRadiansPerSecond = RotationalSpeedFast * rightX;
      } else {
        omegaRadiansPerSecond = RotationalSpeed * rightX;
      }

      SmartDashboard.putBoolean("Omega Corr", false);

    }

    SmartDashboard.putNumber("Drive Omega", omegaRadiansPerSecond);

    // Convert inputs to chassis speeds
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond,
        omegaRadiansPerSecond, Rotation2d.fromDegrees(toWPIAngle(getGyroAngleField())));

    // Run the swerve modules based on current status
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband
        && Math.abs(rightX) < joystickDeadband) {

      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();

    } else {

      if (!Mutables.isParked) {

        driveRobot(fieldSpeeds);

      }

    }

  }

  /**
   * 
   * Drive the robot
   * 
   * @param robotSpeeds A set of chassis speeds
   * 
   */
  public void driveRobot(ChassisSpeeds robotSpeeds) {

    if (againstFront() && robotSpeeds.vxMetersPerSecond > 0) robotSpeeds.vxMetersPerSecond = 0;
    if (againstBack() && robotSpeeds.vxMetersPerSecond < 0) robotSpeeds.vxMetersPerSecond = 0;

    // Convert chassis speeds to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(robotSpeeds);

    // Get module states
    frontLeftState = moduleStates[0];
    frontRightState = moduleStates[1];
    backLeftState = moduleStates[2];
    backRightState = moduleStates[3];

    // Get calculated module angles
    frontLeftAngle = frontLeftState.angle.getDegrees();
    frontRightAngle = frontRightState.angle.getDegrees();
    backLeftAngle = backLeftState.angle.getDegrees();
    backRightAngle = backRightState.angle.getDegrees();

    // Send new settings to swerve wheels as long as we aren't parked
    if (!Mutables.isParked) {

      leftFront.drive(frontLeftState.speedMetersPerSecond, fromWPIAngle(frontLeftAngle));
      rightFront.drive(frontRightState.speedMetersPerSecond, fromWPIAngle(frontRightAngle));
      leftBack.drive(backLeftState.speedMetersPerSecond, fromWPIAngle(backLeftAngle));
      rightBack.drive(backRightState.speedMetersPerSecond, fromWPIAngle(backRightAngle));

    }

    // Check for collision
    double currLinearAccelX = gyro.getWorldLinearAccelX();
    double currentJerkX = currLinearAccelX - lastLinearAccelX;
    lastLinearAccelX = currLinearAccelX;
    double currLinearAccelY = gyro.getWorldLinearAccelY();
    double currentJerkY = currLinearAccelY - lastLinearAccelY;
    lastLinearAccelY = currLinearAccelY;

    if ((Math.abs(currentJerkX) > kCollisionThresholdDeltaG) || (Math.abs(currentJerkY) > kCollisionThresholdDeltaG)) {

      Mutables.impactDetected = true;

    }

    // Send critical values to SmartDashboard for troubleshooting / tuning
    SmartDashboard.putNumber("LF WPI Ang", fromWPIAngle(frontLeftAngle));
    SmartDashboard.putNumber("RF WPI Ang", fromWPIAngle(frontRightAngle));
    SmartDashboard.putNumber("LB WPI Ang", fromWPIAngle(backLeftAngle));
    SmartDashboard.putNumber("RB WPI Ang", fromWPIAngle(backRightAngle));

  }

  /**
   * 
   * Drive the robot
   * 
   * @param robotSpeeds A set of chassis speeds
   * 
   */
  public void driveRobotAuto(ChassisSpeeds robotSpeeds) {

    // Convert chassis speeds to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(robotSpeeds);

    // Get calculated module angles
    frontLeftAngle = moduleStates[0].angle.getDegrees();
    frontRightAngle = moduleStates[1].angle.getDegrees();
    backLeftAngle = moduleStates[2].angle.getDegrees();
    backRightAngle = moduleStates[3].angle.getDegrees();

    // Correct negative angles to be within 0 to 360
    backRightAngle = fromWPIAngle(backRightAngle);
    backLeftAngle = fromWPIAngle(backLeftAngle);
    frontRightAngle = fromWPIAngle(frontRightAngle);
    frontLeftAngle = fromWPIAngle(frontLeftAngle);

    if (!Mutables.isParked) {

      leftFront.drive(moduleStates[0].speedMetersPerSecond, frontLeftAngle);
      rightFront.drive(moduleStates[1].speedMetersPerSecond, frontRightAngle);
      leftBack.drive(moduleStates[2].speedMetersPerSecond, backLeftAngle);
      rightBack.drive(moduleStates[3].speedMetersPerSecond, backRightAngle);

    }

    // Check for collision
    double currLinearAccelX = gyro.getWorldLinearAccelX();
    double currentJerkX = currLinearAccelX - lastLinearAccelX;
    lastLinearAccelX = currLinearAccelX;
    double currLinearAccelY = gyro.getWorldLinearAccelY();
    double currentJerkY = currLinearAccelY - lastLinearAccelY;
    lastLinearAccelY = currLinearAccelY;

    if ((Math.abs(currentJerkX) > kCollisionThresholdDeltaG) || (Math.abs(currentJerkY) > kCollisionThresholdDeltaG)) {

      Mutables.impactDetected = true;

    }

  }

  /**
   * 
   * Stop all modules
   * 
   */
  public void stopDrive() {

    leftFront.stop();
    rightFront.stop();
    leftBack.stop();
    rightBack.stop();

  }

  /**
   * 
   * Get a smoothed gyro angle
   * 
   * @return Gyro angle in degrees (0 to 360)
   * 
   */
  @SuppressWarnings("unused")
  public double getGyroAngle() {
    if (!disableNavx && gyro.isConnected()) {
      double angle = gyro.getAngle() % 360;
      SmartDashboard.putString("Gyro Used", "NavX");
      SmartDashboard.putNumber("Gyro Raw", angle);
      return angle;
    } else if (fallbackGyro.isConnected()) {
      double angle = fallbackGyro.getAngle() % 360;
      SmartDashboard.putString("Gyro Used", "ADXRS450");
      SmartDashboard.putNumber("Gyro Raw", angle);
      return angle;
    } else {
      SmartDashboard.putString("Gyro Used", "None");
      SmartDashboard.putNumber("Gyro Raw", 0);
      return 0;
    }
  }

  /**
   * 
   * Get a smoothed gyro angle
   * 
   * @return Gyro angle in degrees (0 to 360)
   * 
   */
  public double getGyroAngleField() {
    return (getGyroAngle() + GyroCorrection) % 360.0;
  }

  /**
   * 
   * Gets a smoothed gyro yaw angle (-180 to 180)
   * 
   * @return Current yaw angle in radians
   * 
   */
  public double getGyroYaw() {
    if (gyro.isConnected()) {
      // Get filtered yaw angle (in degrees)
      // Negate value to be consistent with WPI coordinate system
      double gyroYaw = -gyro.getYaw();
      // Make sure we don't see -180
      if (gyroYaw == -180.0) {
        gyroYaw = 180.0;
      }
      // Return yaw angle
      return gyroYaw;
    }

    return 0;
  }

  /**
   * 
   * Generates a Rotation2d object from current gyro angle
   * 
   * @return Rotation2d for gyro angle
   */
  public Rotation2d getGyroRotation2d() {

    return new Rotation2d(Math.toRadians(toWPIAngle(getGyroAngle())));

  }

  /**
   * 
   * Get the current yaw rate of the gyro
   * 
   * @return Yaw rate in radians/second
   * 
   */
  public double getGyroYawRate() {
    return gyro.isConnected() ? -gyro.getRate() : fallbackGyro.isConnected() ? fallbackGyro.getRate() : 0;
  }

  /**
   * 
   * Reset current gyro heading to zero
   * 
   */
  public void zeroGyro() {

    gyro.reset();
    fallbackGyro.reset();

  }

  /**
   * 
   * Zero the encoders
   * 
   */
  public void zeroEncoders() {

    leftFront.zeroEncoder();
    rightFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightBack.zeroEncoder();

  }

  /**
   * 
   * Gets encoder value for left front drive motor
   * 
   * @return Raw encoder position
   * 
   */
  public double getLeftFrontDriveEncoder() {

    return leftFront.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for right front drive motor
   * 
   * @return Raw encoder position
   * 
   */
  public double getRightFrontDriveEncoder() {

    return rightFront.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for left back drive motor
   * 
   * @return Raw encoder position
   * 
   */
  public double getLeftBackDriveEncoder() {

    return leftBack.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for right back drive motor
   * 
   * @return Raw encoder position
   * 
   */
  public double getRightBackDriveEncoder() {

    return rightBack.getDriveEncoderPosition();

  }

  /**
   * 
   * Park the bot so it doesn't move
   * 
   */
  public void parkBot() {
    leftFront.drive(0, 135);
    rightFront.drive(0, 225);
    leftBack.drive(0, 45);
    rightBack.drive(0, 315);
  }

  /**
   * 
   * Set the wheel angles to the desired starting position
   * 
   */
  public void homeDriveWheels() {
    leftFront.drive(0, 0);
    rightFront.drive(0, 0);
    leftBack.drive(0, 0);
    rightBack.drive(0, 0);
  }

  /**
   * 
   * Reset the encoder positions for all drive modules
   * 
   */
  public void resetDistance() {

    leftFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightFront.zeroEncoder();
    rightBack.zeroEncoder();

  }

  /**
   * 
   * Calculate the drive distance for robot by
   * averaging distance of each module
   * 
   * @return Distance driven in meters
   * 
   */
  public double calculateDriveDistance() {

    double distance = (leftFront.getDistance() + rightFront.getDistance() + leftBack.getDistance()
        + rightBack.getDistance()) / 4.0;

    SmartDashboard.putNumber("Distance", distance);

    return distance;

  }

  /**
   * 
   * Gets the current position of all swerve modules
   * 
   * @return Array of module positions
   */
  public SwerveModulePosition[] getModulePositions() {

    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();

    return positions;

  }

  /**
   * 
   * Gets the current states of all swerve modules
   * 
   * @return Array of module positions
   */
  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();

    return states;

  }

  public ChassisSpeeds getSpeeds() {

    return kinematics.toChassisSpeeds(getModuleStates());

  }

  public ChassisSpeeds getRobotRelativeSpeeds() {

    return kinematics.toChassisSpeeds(getModuleStates());

  }

  /**
   * 
   * Gets the current pose of the robot in meters
   * 
   * @return Current pose as a Pose2d object
   */
  public Pose2d getPose() {

    return odometry.getPoseMeters();

  }

  /**
   * 
   * Sets the robot pose to a specified pose
   * 
   */
  public void resetPose(Pose2d pose) {

    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);

    SmartDashboard.putString("Starting Pose", getPose().toString());

  }

  /**
   * 
   * Convert a WPI angle (+/-180) to a gyro angle (0-360)
   * 
   * @param angle The WPI angle to convert
   * @return A gyro angle
   * 
   */
  public double fromWPIAngle(double angle) {
    if (angle > 0) {
      angle = (180 - angle) + 180;
    } else if (angle < 0) {
      angle = -angle;
    }
    return angle;
  }

  /**
   * 
   * Convert a gvyro angle (0-360) into a WPI angle (+/-180)
   * 
   * @param angle The gyro angle to convert
   * @return A WPI based angle
   * 
   */
  public double toWPIAngle(double angle) {
    if (angle >= 180) {
      angle = -(angle - 360);
    } else if (angle > 0 && angle < 180) {
      angle = -angle;
    }
    return angle;
  }

  /**
   * 
   * Determines an optional path target rotation override based on seeing
   * the speaker AprilTag
   * 
   * @return a new Rotation2d
   * 
   */
  public Optional<Rotation2d> getTargetRotationOverride() {
    return Optional.empty();
  }

  /**
   * 
   * Get the measured distance for the left front swerve module's laserCAN.
   * 
   * @return the distance in meters
   * 
   */
  public double getLeftFrontLaser() {
    return leftFront.getLaserDistance();
  }

  /**
   * 
   * Get the measured distance for the right front swerve module's laserCAN.
   * 
   * @return the distance in meters
   * 
   */
  public double getRightFrontLaser() {
    return rightFront.getLaserDistance();
  }

  /**
   * 
   * Get the measured distance for the left front swerve module's laserCAN.
   * 
   * @return the distance in meters
   * 
   */
  public double getLeftBackLaser() {
    return leftBack.getLaserDistance();
  }

  /**
   * 
   * Get the measured distance for the right front swerve module's laserCAN.
   * 
   * @return the distance in meters
   * 
   */
  public double getRightBackLaser() {
    return rightBack.getLaserDistance();
  }

  /**
   * Stop driving the robot
   * 
   * @return Command to stop driving
   */
  public Command stopDriving() {
    return runOnce(() -> stopDrive());
  }

  public static boolean flipPaths() {
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    // return alliance.get() == DriverStation.Alliance.Red;
    // }

    return false;
  }

  public Command pathfindTo(Pose2d pose) {
    try {
      RobotConfig ppConfig = RobotConfig.fromGUISettings();
      return new PathfindingCommand(
        pose,
        new PathConstraints(1.0, 1.0, 3.5 * Math.PI, 4.0 * Math.PI),
        this::getPose,
        this::getRobotRelativeSpeeds,
        (speeds, feeds) -> driveRobotRelativePP(speeds),
        new PPHolonomicDriveController(
            translationConstants,
            rotationConstants),
        ppConfig,
        this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      return Commands.none();
    }
  }

  public Command pathfindToNearest(Pose2d... poses) {
    var map = new HashMap<>(Arrays.stream(poses).collect(Collectors.toMap(Optional::of, this::pathfindTo)));
    map.put(Optional.empty(), Commands.print("No available path!"));
    return Commands.select(map, () -> {
      var pose = getPose().getTranslation();
      var best = Arrays.stream(poses).min(Comparator.comparingDouble(p -> p.getTranslation().getDistance(pose)));
      System.out.println(best);
      return best;
    });
  }
}
