// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;

import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

/**
 * Define a SwerveWheel object
 */
public class SwerveWheel2 extends SubsystemBase {

  // Declare constants
  private final double ANGLE_DEADBAND = 0.001; // Deadband for the angle motor. Values smaller than this are
                                               // rounded to 0.
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. This works in the same way as the
                                               // angle one.
  private final double WHEEL_DIAMETER = 0.1016; // diameter in meters
  private final String CANBUS_NAME = "rio";

  // Declare motor variables
  private TalonFX swerveDriveMotor;
  private TalonFX swerveAngleMotor;
  private CANcoder canCoder;
  private VelocityDutyCycle swerveDriveVel;
  private DutyCycleOut swerveAngleOut;
  private DutyCycleOut swerveDriveOut;

  // Declare controller variables
  private PIDController wpiPIDController;
  private double angleSpeedLimiter;
  private double kP_AngleController;
  private double kI_AngleController;
  private double kD_AngleController;

  // Declare sensor variables
  private LaserCan laser;

  // Declare Phoenix PID controller gains
  private double drive_kG;
  private double drive_kS;
  private double drive_kV;
  private double drive_kA;
  private double drive_kP;
  private double drive_kI;
  private double drive_kD;

  // Declare state variables
  private int wheelID;

  // Declare general variables
  private String moduleName;

  // Declare velocity control variables
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /**
   * 
   * Creates a new SwerveWheel
   * 
   * @param name         Name of this swerve module
   * @param driveMotorID CAN ID for the drive motor
   * @param angleMotorID CAN ID for the angle motor
   * @param CANCoderID   CAN ID for the angle encoder
   * @param laserCANID   CAN ID for the laser range finder
   * 
   */
  public SwerveWheel2(String name, int driveMotorID, int angleMotorID, int CANCoderID, int laserCANID) {

    // Set variables
    moduleName = name;

    // Set wheel ID
    wheelID = CANCoderID / 3;

    // Set WPI angle controller gains
    angleSpeedLimiter = DriveConstants.angleLimiters[wheelID - 1];
    kP_AngleController = DriveConstants.anglePIDkPs[wheelID - 1];
    kI_AngleController = DriveConstants.anglePIDkIs[wheelID - 1];
    kD_AngleController = DriveConstants.anglePIDkDs[wheelID - 1];

    // Set Phoenix drive PID controller gains
    drive_kG = 0.0;
    drive_kS = 0.1;
    drive_kV = 0.1;
    drive_kA = 0.0;
    drive_kP = 0.1;
    drive_kI = 0.0;
    drive_kD = 0.0;

    // Put PID constants on SmartDashboard for testing
    SmartDashboard.putNumber(moduleName + " kP", kP_AngleController);
    SmartDashboard.putNumber(moduleName + " kI", kI_AngleController);
    SmartDashboard.putNumber(moduleName + " kD", kD_AngleController);

    // Create motors
    swerveDriveMotor = new TalonFX(driveMotorID, CANBUS_NAME);
    swerveAngleMotor = new TalonFX(angleMotorID, CANBUS_NAME);

    // Create CAN encoder
    assert HAL.initialize(500, 0);
    canCoder = new CANcoder(CANCoderID);

    // Create and configure laser range finder
    laser = new LaserCan(laserCANID);
    try {
      laser.setRangingMode(LaserCan.RangingMode.SHORT);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println(moduleName + " LaserCAN configuration failed. Error: " + e.toString());
      DriverStation.reportError(moduleName + " LaserCAN configuration failed.", false);
    }

    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID, CANCoderID);

    // Create motor outputs and initialize to 0%
    swerveDriveVel = new VelocityDutyCycle(0);
    swerveDriveOut = new DutyCycleOut(0);
    swerveDriveMotor.setControl(swerveDriveVel);
    swerveAngleOut = new DutyCycleOut(0);
    swerveAngleMotor.setControl(swerveAngleOut);

    // Initialize PID controller
    wpiPIDController = new PIDController(kP_AngleController, kI_AngleController, kD_AngleController);
    wpiPIDController.setTolerance(1.5, 5);
  }

  /**
   * 
   * Initialize motors
   * 
   * @param driveMotorID CAN ID for the drive motor
   * @param angleMotorID CAN ID for the angle motor
   * @param CANCoderID   CAN ID for the angle encoder
   * 
   */
  private void InitSwerveMotors(int driveMotorID, int angleMotorID, int CANCoderID) {

    // Create drive motor configuration
    var driveConfigs = new TalonFXConfiguration();

    // Set drive motor output configuration
    var driveOutputConfigs = driveConfigs.MotorOutput;
    driveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    driveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set drive motor current limits
    var driveLimitConfig = driveConfigs.CurrentLimits;
    driveLimitConfig.StatorCurrentLimitEnable = true;
    driveLimitConfig.StatorCurrentLimit = 100;

    // Set drive motor feedback sensor
    var driveSensorConfig = driveConfigs.Feedback;
    driveSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set drive motor PID constants
    var slot0Configs = driveConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply drive motor configuration and initialize position to 0
    StatusCode driveStatus = swerveDriveMotor.getConfigurator().apply(driveConfigs, 0.050);
    if (!driveStatus.isOK()) {
      System.out.println(
          "Could not apply drive motor configs for wheel: " + wheelID + ". Error code: " + driveStatus.toString());
      DriverStation.reportError("Could not apply drive motor configs for " + moduleName + " wheel.", false);
    } else {
      System.out.println(
          "Successfully applied drive motor configs for wheel: " + wheelID + ". Error code: " + driveStatus.toString());
    }
    swerveDriveMotor.getConfigurator().setPosition(0);

    // Create angle motor configuration
    var angleConfigs = new TalonFXConfiguration();

    // Set angle motor output configuration
    var angleOutputConfigs = angleConfigs.MotorOutput;
    angleOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;  //inverted (normally counterclockwise positive)
    angleOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    angleOutputConfigs.withDutyCycleNeutralDeadband(ANGLE_DEADBAND);

    // Apply angle motor configuration and initialize position to 0
    StatusCode angleStatus = swerveAngleMotor.getConfigurator().apply(angleConfigs, 0.050);
    if (!angleStatus.isOK()) {
      System.out.println("Could not apply angle motor configs for " + moduleName + " wheel." + ". Error code: "
          + angleStatus.toString());
      DriverStation.reportError("Could not apply angle motor configs for " + moduleName + " wheel.", false);
    } else {
      System.out.println(
          "Successfully applied angle motor configs for wheel: " + wheelID + ". Error code: " + angleStatus.toString());
    }

  }

  /**
   * 
   * Periodically update swerve wheel status
   * 
   */
  @Override
  public void periodic() {

    // Update status of drive motor
    SmartDashboard.putNumber(moduleName + " Drive Motor Volts", swerveDriveMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Drive Motor Amps", swerveDriveMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Drive Motor Position", swerveDriveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Drive Motor Velocity", swerveDriveMotor.getVelocity().getValueAsDouble());

    // Update status of angle motor
    SmartDashboard.putNumber(moduleName + " Angle Motor Volts", swerveAngleMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Angle Motor Amps", swerveAngleMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Angle Motor Position", swerveAngleMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Angle Motor Velocity", swerveAngleMotor.getVelocity().getValueAsDouble());

    // Update status of CANCoder
    SmartDashboard.putNumber(moduleName + " CANCoder Position", canCoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " CANCoder Velocity", canCoder.getVelocity().getValueAsDouble());

    // Update wheel status
    SmartDashboard.putNumber(moduleName + " Wheel Speed", getWheelSpeed());
    SmartDashboard.putNumber(moduleName + " Wheel Dist", getDistance());

    // Update the laser distance
    SmartDashboard.putNumber(moduleName + " Laser Distance", getLaserDistance());

  }

  /**
   * 
   * Drive this wheel module at the specified speed and angle
   * 
   * @param speed Speed for the drive motor
   * @param angle Wheel angle for this module
   * 
   */
  public void drive(double speed, double angle) {

    // Normalize target to have a max value of 1
    double normAngle = angle / 360.0;
    if (normAngle == 1.0) {
      normAngle = 0.0;
    }

    // Get current wheel angle
    double encoderAngle = canCoder.getAbsolutePosition().getValueAsDouble();

    // Optimize angle motor rotation direction and distance
    double error = (encoderAngle - normAngle + 0.5) % 1 - 0.5;
    if (error < -0.5)
      error += 1.0;
    else if (error > 0.5)
      error -= 1.0;
    if (error < -0.25) {
      error += 0.5;
      speed = -speed;
    } else if (error > 0.25) {
      error -= 0.5;
      speed = -speed;
    }

    // Calculate angle motor output demand
    double output = wpiPIDController.calculate(error, 0);

    // Apply speed limiter to angle motor output demand
    double angleSpeed = output * angleSpeedLimiter;

    // Capping angleSpeed to max that the motor can take, from -1 to 1
    if (angleSpeed > 1) {
      angleSpeed = 1;
    } else if (angleSpeed < -1) {
      angleSpeed = -1;
    }

    // Calculate wheel velocity
    double motorVelocity = (speed / (Math.PI * WHEEL_DIAMETER) * DriveConstants.kGearRatio);
    double motorVelocityRPM = motorVelocity * 60;

    // Set outputs for angle and drive motors
    swerveAngleOut.Output = angleSpeed;
    swerveAngleMotor.setControl(swerveAngleOut);
    swerveDriveMotor.setControl(m_request.withVelocity(motorVelocity));

    // Send critical values to SmartDashboard for troubleshooting / tuning
    SmartDashboard.putNumber(moduleName + " req speed", speed);
    SmartDashboard.putNumber(moduleName + " req angle", angle);
    SmartDashboard.putNumber(moduleName + " norm angle", normAngle);
    SmartDashboard.putNumber(moduleName + " angle error", error);
    SmartDashboard.putNumber(moduleName + " PID output", output);
    SmartDashboard.putNumber(moduleName + " angle speed", angleSpeed);
    SmartDashboard.putNumber(moduleName + " V target", motorVelocity);
    SmartDashboard.putNumber(moduleName + " target RPM", motorVelocityRPM);
    SmartDashboard.putNumber(moduleName + " V actual", getDriveEncoderVelocity());

  }

  /**
   *
   * Get the encoder position value of the drive motor
   * 
   * @return Current position (number of pulses) of the drive encoder
   * 
   */
  public double getDriveEncoderPosition() {

    var drivePosSignal = swerveDriveMotor.getPosition();
    drivePosSignal.refresh();
    return drivePosSignal.getValueAsDouble();

  }

  /**
   * 
   * Get the encoder velocity value of the drive motor
   * 
   * @return Current velocity (rotations/second) of the drive encoder
   * 
   */
  public double getDriveEncoderVelocity() {

    var driveVelSignal = swerveDriveMotor.getVelocity();
    driveVelSignal.refresh();
    return driveVelSignal.getValueAsDouble();

  }

  /**
   * 
   * Zeros the encoder for the drive motor
   * 
   */
  public void zeroEncoder() {

    swerveDriveMotor.getConfigurator().setPosition(0);

  }

  /**
   * 
   * Stop all motors
   * 
   */
  public void stop() {

    swerveDriveMotor.stopMotor();
    swerveAngleMotor.stopMotor();
  }

  /**
   * 
   * Calculate the drive distance for this module since last reset
   * 
   * @return Drive distance in meters
   * 
   */
  public double getDistance() {

    return (WHEEL_DIAMETER * Math.PI * getDriveEncoderPosition()) / DriveConstants.kGearRatio;

  }

  /**
   * 
   * Calculate the drive velocity for this module
   * 
   * @return Drive velocity in meters/second
   * 
   */
  public double getWheelSpeed() {

    return (WHEEL_DIAMETER * Math.PI * getDriveEncoderVelocity()) / DriveConstants.kGearRatio;

  }

  /**
   * 
   * Get the current state of the module
   * 
   * @return Current state of this module
   * 
   */
  public SwerveModuleState getState() {

    return new SwerveModuleState(getWheelSpeed(),
        new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition().getValueAsDouble() * 360.0))));

  }

  /**
   * 
   * Get the current position of the module
   * 
   * @return Current position of this module
   * 
   */
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(getDistance(),
        new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition().getValueAsDouble() * 360.0))));

  }

  /**
   * 
   * Get the current distance measurement from the LaserCAN
   * 
   * @return  Distance in meters (returns -1 if measurement error)
   */
  public double getLaserDistance() {
    LaserCan.Measurement distance = laser.getMeasurement();
    if (distance != null && distance.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return distance.distance_mm / 1000.0;
    } else {
      return -1.0;
    }
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
    if (angle > 180) {
      angle = -(angle - 360);
    } else if (angle > 0 && angle <= 180) {
      angle = -angle;
    }
    return angle;
  }

}
