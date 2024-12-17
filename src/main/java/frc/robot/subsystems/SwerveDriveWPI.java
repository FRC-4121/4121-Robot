// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Utils;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControlConstants;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveDriveWPI extends SubsystemBase {
  // Swerve drive wheels
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;

  // Swerve kinematics and odometry objects
  private SwerveDriveKinematics kinematics;
  private Translation2d leftFrontTranslation;
  private Translation2d leftBackTranslation;
  private Translation2d rightFrontTranslation;
  private Translation2d rightBackTranslation;
  private SwerveDriveOdometry odometry;

  // Declare swerve module inputs
  private double frontLeftAngle;
  private double backLeftAngle;
  private double frontRightAngle;
  private double backRightAngle;

  // Declare NavX gyro Objects
  private AHRS gyro;
  private MedianFilter gyro_filter;
  private MedianFilter yaw_filter;

  private double joystickDeadband;

  // Declare speed variables
  private double vxMetersPerSecond;
  private double vyMetersPerSecond;

  // Collision detection state
  private double lastLinearAccelX;
  private double lastLinearAccelY;

  // Declare PID controller
  private PIDController wpiPIDController;

  // Declare 2d Field
  private Field2d field;

  private static final int leftFrontDriveId = 1;
  private static final int leftFrontAngleId = 2;
  private static final int leftFrontCoderId = 3;

  private static final int rightFrontDriveId = 4;
  private static final int rightFrontAngleId = 5;
  private static final int rightFrontCoderId = 6;

  private static final int rightBackDriveId = 7;
  private static final int rightBackAngleId = 8;
  private static final int rightBackCoderId = 9;

  private static final int leftBackDriveId = 10;
  private static final int leftBackAngleId = 11;
  private static final int leftBackCoderId = 12;

  /**
   * 
   * Creates a new SwerveDrive
   * 
   */
  public SwerveDriveWPI() {
    // Initialize new swerve modules
    leftFront = new SwerveWheel(
        SwerveWheel.baseConfig.clone()
            .withName("LF")
            .withIds(leftFrontDriveId, leftFrontAngleId, leftFrontCoderId));
    leftBack = new SwerveWheel(
        SwerveWheel.baseConfig.clone()
            .withName("LB")
            .withIds(leftBackDriveId, leftBackAngleId, leftBackCoderId));
    rightFront = new SwerveWheel(
        SwerveWheel.baseConfig.clone()
            .withName("RF")
            .withIds(rightFrontDriveId, rightFrontAngleId, rightFrontCoderId));
    rightBack = new SwerveWheel(
        SwerveWheel.baseConfig.clone()
            .withName("RB")
            .withIds(rightBackDriveId, rightBackAngleId, rightBackCoderId));

    // Initialize swerve kinematics objects
    leftFrontTranslation = new Translation2d(0.297, 0.288);// X-0.297, Y-0.288
    leftBackTranslation = new Translation2d(-0.297, 0.288);// 0.229,0.292 for last years bot
    rightFrontTranslation = new Translation2d(0.297, -0.288);
    rightBackTranslation = new Translation2d(-0.297, -0.288);
    kinematics = new SwerveDriveKinematics(leftFrontTranslation, rightFrontTranslation, leftBackTranslation,
        rightBackTranslation);

    // Initialize NavX gyro
    gyro = new AHRS(SPI.Port.kMXP);

    // gyro.calibrate();
    gyro.reset();
    gyro.resetDisplacement();
    // gyro.setAngleAdjustment(270);

    // Initialize gyro filter
    gyro_filter = new MedianFilter(Constants.FILTER_WINDOW_SIZE);
    yaw_filter = new MedianFilter(Constants.FILTER_WINDOW_SIZE);

    // Initialize misc variables
    joystickDeadband = 0.05;
    lastLinearAccelX = 0.0;
    lastLinearAccelY = 0.0;

    // Create PID controller
    wpiPIDController = new PIDController(DriveConstants.kAnglePIDkp, DriveConstants.kAnglePIDki,
        DriveConstants.kAnglePIDkd);
    wpiPIDController.setTolerance(1.0, 5);

    // Initialize swerve odometry object
    odometry = new SwerveDriveOdometry(kinematics, getGyroRotation2d(), getModulePositions());

    // Initialize 2d field
    field = new Field2d();

    // Configure AutoBuilder
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getSpeeds,
        this::driveRobotRelative,
        Constants.Swerve.pathFollowerConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          /*
           * var alliance = DriverStation.getAlliance();
           * if (alliance.isPresent()) {
           * return alliance.get() == DriverStation.Alliance.Red;
           * }
           * return false;
           */
          return !Constants.blueAlliance;
        },
        this);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
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
    if (zeroGyro != 1) {
      SmartDashboard.putNumber("Zero Gyro", 0);
      // gyro.calibrate();
      zeroGyro();
    }

    // Update robot odometry
    odometry.update(getGyroRotation2d(), getModulePositions());
    SmartDashboard.putString("Pose", getPose().toString());
    field.setRobotPose(getPose());
  }

  /**
   * 
   * Drive the robot relative to robot coordinate system
   * 
   * @param robotRelativeSpeeds A set of chassis speeds
   * 
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
    // 0.02);
    driveRobotAuto(robotRelativeSpeeds);

    System.out.println("Robot Relative Drive");
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
    vxMetersPerSecond = -(leftY * DriveConstants.LinearSpeed);
    vyMetersPerSecond = (leftX * DriveConstants.LinearSpeed);

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < ControlConstants.kJoystickTolerance) {
      double pidOutput = wpiPIDController.calculate(Math.toRadians(getGyroYawRate()), 0.0);
      omegaRadiansPerSecond = DriveConstants.RotationalSpeed * pidOutput;
    } else {
      omegaRadiansPerSecond = DriveConstants.RotationalSpeed * rightX;
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
    } else if (!Constants.isParked)
      driveRobot(relativeSpeeds);
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
        Rotation2d.fromDegrees(getGyroAngle()));
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
    vxMetersPerSecond = -(leftY * DriveConstants.LinearSpeed);
    vyMetersPerSecond = (leftX * DriveConstants.LinearSpeed);

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < ControlConstants.kJoystickTolerance) {
      double pidOutput = wpiPIDController.calculate(Math.toRadians(getGyroYawRate()), 0.0);
      omegaRadiansPerSecond = DriveConstants.RotationalSpeed * pidOutput;
    } else {
      omegaRadiansPerSecond = DriveConstants.RotationalSpeed * rightX;
    }

    // Convert inputs to chassis speeds
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond,
        omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroAngle()));

    // Run the swerve modules based on current status
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband
        && Math.abs(rightX) < joystickDeadband) {
      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();
    } else if (!Constants.isParked) {
      driveRobot(fieldSpeeds);
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
    // Convert chassis speeds to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(robotSpeeds);

    // Get calculated module angles
    frontLeftAngle = moduleStates[2].angle.getDegrees();
    frontRightAngle = moduleStates[3].angle.getDegrees();
    backLeftAngle = moduleStates[0].angle.getDegrees();
    backRightAngle = moduleStates[1].angle.getDegrees();

    // Correct negative angles to be within 0 to 360
    if (backRightAngle < 0)
      backRightAngle += 360;
    if (backLeftAngle < 0)
      backLeftAngle += 360;
    if (frontRightAngle < 0)
      frontRightAngle += 360;
    if (frontLeftAngle < 0)
      frontLeftAngle += 360;

    if (!Constants.isParked) {
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

    if ((Math.abs(currentJerkX) > DriveConstants.kCollisionThresholdDeltaG)
        || (Math.abs(currentJerkY) > DriveConstants.kCollisionThresholdDeltaG)) {
      Constants.impactDetected = true; // this never gets set to false?
    }
  }

  /**
   * 
   * Drive the robot
   * 
   * @param robotSpeeds A set of chassis speeds
   * @deprecated use driveRobot() instead
   */
  @Deprecated
  public void driveRobotAuto(ChassisSpeeds robotSpeeds) {

    System.out.println("drive robot auto");

    // Convert chassis speeds to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(robotSpeeds);

    // Get calculated module angles
    frontLeftAngle = moduleStates[0].angle.getDegrees();
    frontRightAngle = moduleStates[1].angle.getDegrees();
    backLeftAngle = moduleStates[2].angle.getDegrees();
    backRightAngle = moduleStates[3].angle.getDegrees();

    // Correct negative angles to be within 0 to 360
    backRightAngle = Utils.fromWPIAngle(backRightAngle);
    backLeftAngle = Utils.fromWPIAngle(backLeftAngle);
    frontRightAngle = Utils.fromWPIAngle(frontRightAngle);
    frontLeftAngle = Utils.fromWPIAngle(frontLeftAngle);

    if (!Constants.isParked) {
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

    if ((Math.abs(currentJerkX) > DriveConstants.kCollisionThresholdDeltaG) || (Math.abs(currentJerkY) > DriveConstants.kCollisionThresholdDeltaG)) {
      Constants.impactDetected = true;
    }
  }

  /**
   * 
   * Moves the robot by calculating the speed and angle for each
   * swerve module based on joystick input
   * 
   * @param leftX  The left-right position of the left joystick. Controls slew.
   *               This is Y direction (left +) in WPI coordinate system.
   * @param leftY  The up-down position of the left joystick. Controls
   *               forward/backward. This is X direction (up +) in WPI coordinate
   *               system.
   * @param rightX The left-right position of the right joystick. Controls
   *               rotation.
   * 
   */
  /*
   * public void drive(double leftX, double leftY, double rightX) {
   * 
   * // Convert joystick positions to linear speeds in meters/second
   * vxMetersPerSecond = -(leftY * LinearSpeed);
   * vyMetersPerSecond = (leftX * LinearSpeed);
   * 
   * // Get rotational speed
   * double omegaRadiansPerSecond = 0.0;
   * if(Math.abs(rightX) < kJoystickTolerance){
   * double pidOutput =
   * wpiPIDController.calculate(Math.toRadians(getGyroYawRate()), 0.0);
   * omegaRadiansPerSecond = RotationalSpeed * pidOutput;
   * } else{
   * omegaRadiansPerSecond = RotationalSpeed * rightX;
   * }
   * 
   * // Create Chassis Speed based on drive mode
   * if (isFieldOriented) {
   * 
   * speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond,
   * vyMetersPerSecond, omegaRadiansPerSecond,
   * Rotation2d.fromDegrees(getGyroAngle()));
   * //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond,
   * vyMetersPerSecond, omegaRadiansPerSecond,
   * Rotation2d.fromDegrees(getGyroYaw()));
   * 
   * }
   * else {
   * 
   * speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond,
   * omegaRadiansPerSecond);
   * 
   * }
   * 
   * // Convert chassis speeds to module states
   * SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
   * 
   * // Get calculated module angles
   * frontLeftAngle = moduleStates[2].angle.getDegrees();
   * frontRightAngle = moduleStates[3].angle.getDegrees();
   * backLeftAngle = moduleStates[0].angle.getDegrees();
   * backRightAngle = moduleStates[1].angle.getDegrees();
   * 
   * // Correct negative angles to be within 0 to 360
   * if (backRightAngle < 0)
   * {
   * backRightAngle = 360 + backRightAngle;
   * }
   * if (backLeftAngle < 0)
   * {
   * backLeftAngle = 360 + backLeftAngle;
   * }
   * if (frontRightAngle < 0)
   * {
   * frontRightAngle = 360 + frontRightAngle;
   * }
   * if (frontLeftAngle < 0)
   * {
   * frontLeftAngle = 360 + frontLeftAngle;
   * }
   * 
   * // Run the swerve modules based on current status
   * if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband
   * && Math.abs(rightX) < joystickDeadband) {
   * 
   * // Stop motors without turning wheels
   * leftFront.stop();
   * rightFront.stop();
   * leftBack.stop();
   * rightBack.stop();
   * 
   * }
   * else {
   * 
   * if (!isParked) {
   * 
   * leftFront.drive(moduleStates[0].speedMetersPerSecond, frontLeftAngle);
   * rightFront.drive(moduleStates[1].speedMetersPerSecond, frontRightAngle);
   * leftBack.drive(moduleStates[2].speedMetersPerSecond, backLeftAngle);
   * rightBack.drive(moduleStates[3].speedMetersPerSecond, backRightAngle);
   * 
   * // Put drive values on dashboard for testing/debugging
   * SmartDashboard.putNumber("Left Front Speed",
   * moduleStates[0].speedMetersPerSecond);
   * SmartDashboard.putNumber("Left Front Angle",
   * moduleStates[0].angle.getDegrees());
   * SmartDashboard.putNumber("Corrected Left Front Angle", frontLeftAngle);
   * SmartDashboard.putNumber("Right Front Speed",
   * moduleStates[1].speedMetersPerSecond);
   * SmartDashboard.putNumber("Right Front Angle",
   * moduleStates[1].angle.getDegrees());
   * SmartDashboard.putNumber("Corrected Right Front Angle", frontRightAngle);
   * SmartDashboard.putNumber("Left Back Speed",
   * moduleStates[2].speedMetersPerSecond);
   * SmartDashboard.putNumber("Left Back Angle",
   * moduleStates[2].angle.getDegrees());
   * SmartDashboard.putNumber("Corrected Left Back Angle", backLeftAngle);
   * SmartDashboard.putNumber("Right Back Speed",
   * moduleStates[3].speedMetersPerSecond);
   * SmartDashboard.putNumber("Right Back Angle",
   * moduleStates[3].angle.getDegrees());
   * SmartDashboard.putNumber("Corrected Right Back Angle", backRightAngle);
   * 
   * }
   * 
   * }
   * 
   * // Check for collision
   * double currLinearAccelX = gyro.getWorldLinearAccelX();
   * double currentJerkX = currLinearAccelX - lastLinearAccelX;
   * lastLinearAccelX = currLinearAccelX;
   * double currLinearAccelY = gyro.getWorldLinearAccelY();
   * double currentJerkY = currLinearAccelY - lastLinearAccelY;
   * lastLinearAccelY = currLinearAccelY;
   * 
   * if ( (Math.abs(currentJerkX) > kCollisionThresholdDeltaG ) ||
   * (Math.abs(currentJerkY) > kCollisionThresholdDeltaG) ) {
   * 
   * impactDetected = true;
   * 
   * }
   * 
   * }
   */

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
  public double getGyroAngle() {
    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    if (correctedGyro < 0)
      correctedGyro += 360;

    // Correct gyro for starting position
    if (Constants.autoPosition == "Left")
      return (correctedGyro + DriveConstants.leftGyroCorrection) % 360;
    if (Constants.autoPosition == "Right")
      return (correctedGyro + DriveConstants.rightGyroCorrection) % 360;

    return correctedGyro;
  }

  /**
   * 
   * Gets a smoothed gyro yaw angle (-180 to 180)
   * 
   * @return Current yaw angle in radians
   * 
   */
  public double getGyroYaw() {
    // Get filtered yaw angle (in degrees)
    // Negate value to be consistent with WPI coordinate system
    double gyroYaw = -gyro_filter.calculate(Math.toRadians(gyro.getYaw()));

    // Make sure we don't see -180
    if (gyroYaw == -180.0)
      gyroYaw = 180.0;

    // Correct angle for starting position
    if (Constants.autoPosition == "Left")
      gyroYaw += DriveConstants.leftGyroCorrection;
    if (Constants.autoPosition == "Right")
      gyroYaw += DriveConstants.rightGyroCorrection;

    // Return yaw angle
    return gyroYaw;
  }

  /**
   * Generates a Rotation2d object from current gyro angle
   * 
   * @return Rotation2d for gyro angle
   */
  public Rotation2d getGyroRotation2d() {
    return new Rotation2d(Math.toRadians(Utils.toWPIAngle(getGyroAngle())));
  }

  /**
   * Get the current yaw rate of the gyro
   * 
   * @return Yaw rate in radians/second
   */
  public double getGyroYawRate() {
    double yawRate = -yaw_filter.calculate(Math.toRadians(gyro.getRate()));
    return yawRate;
  }

  /**
   * Reset current gyro heading to zero
   */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * Zero the encoders
   */
  public void zeroEncoders() {
    leftFront.zeroEncoder();
    rightFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightBack.zeroEncoder();
  }

  /**
   * Park the bot so it doesn't move
   */
  public void parkBot() {
    leftFront.drive(0, 135);
    rightFront.drive(0, 225);
    leftBack.drive(0, 45);
    rightBack.drive(0, 315);
  }

  /**
   * Reset the encoder positions for all drive modules
   */
  public void resetDistance() {
    leftFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightFront.zeroEncoder();
    rightBack.zeroEncoder();
  }

  /**
   * Calculate the drive distance for robot by
   * averaging distance of each module
   * 
   * @return Distance driven in meters
   */
  public double calculateDriveDistance() {
    double distance = (leftFront.getDistance() + rightFront.getDistance() + leftBack.getDistance()
        + rightBack.getDistance()) / 4.0;

    SmartDashboard.putNumber("Distance", distance);

    return distance;
  }

  /**
   * Gets the current position of all swerve modules
   * 
   * @return Array of module positions with the order LF, RF, LB, RB
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
   * Gets the current states of all swerve modules
   * 
   * @return Array of module states with the order LF, RF, LB, RB
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
   * Gets the current pose of the robot in meters
   * 
   * @return Current pose as a Pose2d object
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Sets the robot pose to a specified pose
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    SmartDashboard.putString("Starting Pose", getPose().toString());
  }

  /**
   * Determines an optional path target rotation override based on seeing
   * the speaker AprilTag
   * 
   * @return the override, if one exists
   */
  public Optional<Rotation2d> getTargetRotationOverride() {
    return Optional.empty();
  }
}
