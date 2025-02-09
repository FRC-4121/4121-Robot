// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import static frc.robot.Constants.*;
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
import edu.wpi.first.wpilibj.DriverStation;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

/**
 * Define a SwerveDrive object
 */
public class SwerveDriveWPI extends SubsystemBase {

  // Declare CAN IDs for swerve wheels
  private final int leftFrontDriveId = 1;
  private final int leftFrontAngleId = 2;
  private final int leftFrontCoderId = 3;

  private final int rightFrontDriveId = 4;
  private final int rightFrontAngleId = 5;
  private final int rightFrontCoderId = 6;

  private final int rightBackDriveId = 10;
  private final int rightBackAngleId = 11;
  private final int rightBackCoderId = 12;

  private final int leftBackDriveId = 7;
  private final int leftBackAngleId = 8;
  private final int leftBackCoderId = 9;

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
  private MedianFilter gyro_filter;
  private MedianFilter yaw_filter;

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

  /**
   * 
   * Creates a new SwerveDrive
   * 
   */
  public SwerveDriveWPI() {

    // Initialize new swerve modules
    leftFront = new SwerveWheel2("LF", leftFrontDriveId, leftFrontAngleId, leftFrontCoderId);
    leftBack = new SwerveWheel2("LB", leftBackDriveId, leftBackAngleId, leftBackCoderId);
    rightFront = new SwerveWheel2("RF", rightFrontDriveId, rightFrontAngleId, rightFrontCoderId);
    rightBack = new SwerveWheel2("RB", rightBackDriveId, rightBackAngleId, rightBackCoderId);

    // Initialize swerve kinematics objects
    // 2025 robot chassis is 30" x 30"
    leftFrontTranslation = new Translation2d(0.311, 0.311);// X-0.297, Y-0.288
    leftBackTranslation = new Translation2d(-0.311, 0.311);// 0.229,0.292 for last years bot
    rightFrontTranslation = new Translation2d(0.311, -0.311);
    rightBackTranslation = new Translation2d(-0.311, -0.311);
    kinematics = new SwerveDriveKinematics(leftFrontTranslation, rightFrontTranslation, leftBackTranslation,
        rightBackTranslation);

    // Initialize NavX gyro
    gyro = new AHRS(NavXComType.kMXP_SPI);

    // gyro.calibrate();
    gyro.reset();
    gyro.resetDisplacement();

    // Initialize gyro filter
    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
    yaw_filter = new MedianFilter(FILTER_WINDOW_SIZE);

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
          this::getSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              translationConstants,
              rotationConstants),
          ppConfig,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

/*             var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
 */            return false;

          },
          this
        );
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

    SmartDashboard.putString("Pose", getPose().toString());

    // Update field position
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
    vxMetersPerSecond = -(leftY * LinearSpeed);
    vyMetersPerSecond = -(leftX * LinearSpeed);

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < kJoystickTolerance) {
      double pidOutput = wpiPIDController.calculate(Math.toRadians(getGyroYawRate()), 0.0);
      omegaRadiansPerSecond = RotationalSpeed * pidOutput;
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

      if (!isParked) {

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
        Rotation2d.fromDegrees(toWPIAngle(getGyroAngle())));
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

    // Get rotational speed
    double omegaRadiansPerSecond = 0.0;
    if (Math.abs(rightX) < kJoystickTolerance) {

      double yawRate = getGyroYawRate();
      double pidOutput = wpiPIDController.calculate(yawRate, 0.0);
      if (vxMetersPerSecond >= 0.5 * LinearSpeed) {
        omegaRadiansPerSecond = RotationalSpeed * pidOutput;
      } else {
        omegaRadiansPerSecond = 0.5 * RotationalSpeed * pidOutput;
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
        omegaRadiansPerSecond, Rotation2d.fromDegrees(toWPIAngle(getGyroAngle())));

    // Run the swerve modules based on current status
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband
        && Math.abs(rightX) < joystickDeadband) {

      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();

    } else {

      if (!isParked) {

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
    if (!isParked) {

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

      impactDetected = true;

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

    System.out.println("drive robot auto");

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

    System.out.println("lf" + frontLeftAngle);
    System.out.println("rf" + frontRightAngle);
    System.out.println("lb" + backLeftAngle);
    System.out.println("rb" + backRightAngle);

    if (!isParked) {

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

      impactDetected = true;

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
  public double getGyroAngle() {

    double correctedGyro = gyro_filter.calculate((gyro.getAngle() + GyroCorrection) % 360.0);
    if (correctedGyro < 0) {
      correctedGyro = 360 + correctedGyro;
    }

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
    if (gyroYaw == -180.0) {
      gyroYaw = 180.0;
    }

    // Return yaw angle
    return gyroYaw;

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

    //double yawRate = -yaw_filter.calculate(Math.toRadians(gyro.getRate()));
    //return -Math.toRadians(gyro.getRate());
    return -gyro.getRate();

  }

  /**
   * 
   * Reset current gyro heading to zero
   * 
   */
  public void zeroGyro() {

    gyro.reset();

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

}
