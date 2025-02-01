// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import static frc.robot.Constants.kGearRatio;
//import static frc.robot.Constants.DriveConstants.swerveDriveSpeedLimiter;
//import static frc.robot.Constants.DriveConstants.kTalonFXPPR;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.*;
import edu.wpi.first.math.kinematics.*;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveWheel2 extends SubsystemBase {

  // Declare constants
  private final int FILTER_WINDOW_SIZE = 10;
  private final double CONFIG_TIMEOUT = 0.01; // Timeout for each configuration, in seconds.
  private final double ANGLE_DEADBAND = 0.001; // Deadband for the angle motor. Values smaller than this are
                                                     // rounded to 0.
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. This works in the same way as the
                                                     // angle one.
  private final double WHEEL_DIAMETER = 0.1016;
  private final double DRIVE_GEAR_RATIO = 8.14;
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

  // Declare Phoenix PID controller gains
  private double phoenix_Kg;
  private double phoenix_Ks;
  private double phoenix_Kv;
  private double phoenix_Ka;
  private double phoenix_Kp;
  private double phoenix_Ki;
  private double phoneix_Kd;

  // Declare state variables
  private int wheelID;

  // Declare general variables
  private String moduleName;

  // Declare velocity control variables
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  //private final VelocityDutyCycle m_request = new VelocityDutyCycle(0).withSlot(0);

  /**
   * 
   *  Creates a new SwerveWheel
   * 
   * @param name  Name of this swerve module
   * @param driveMotorID  CAN ID for the drive motor
   * @param angleMotorID  CAN ID for the angle motor
   * @param CANCoderID  CAN ID for the angle encoder
   * 
   */
  public SwerveWheel2(String name, int driveMotorID, int angleMotorID, int CANCoderID) {
    
    // Set variables
    moduleName = name;

    // Set wheel ID
    wheelID = CANCoderID / 3;

    // Set WPI angle controller gains
    angleSpeedLimiter = angleLimiters[wheelID - 1];
    kP_AngleController = anglePIDkPs[wheelID - 1];
    kI_AngleController = anglePIDkIs[wheelID - 1];
    kD_AngleController = anglePIDkDs[wheelID - 1];

    // Set Phoenix drive PID controller gains
    phoenix_Kg = 0.0;
    phoenix_Ks = 0.1;
    phoenix_Kv = 0.1;
    phoenix_Ka = 0.0;
    phoenix_Kp = 0.1;
    phoenix_Ki = 0.0;
    phoneix_Kd = 0.0;

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
    wpiPIDController.setTolerance(1.5,5);

  } 

  /**
   * 
   *  Initialize motors
   * 
   * @param driveMotorID  CAN ID for the drive motor
   * @param angleMotorID  CAN ID for the angle motor
   * @param CANCoderID  CAN ID for the angle encoder
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

    // Set drive motor feedback sensor
    var driveSensorConfig = driveConfigs.Feedback;
    driveSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set drive motor PID constants
    var slot0Configs = driveConfigs.Slot0;
    slot0Configs.kG = phoenix_Kg;
    slot0Configs.kS = phoenix_Ks;
    slot0Configs.kV = phoenix_Kv;
    slot0Configs.kA = phoenix_Ka;
    slot0Configs.kP = phoenix_Kp;
    slot0Configs.kI = phoenix_Ki;
    slot0Configs.kD = phoneix_Kd;

    // Apply drive motor configuration and initialize position to 0
    //StatusCode status = StatusCode.StatusCodeNotInitialized;
    StatusCode driveStatus = swerveDriveMotor.getConfigurator().apply(driveConfigs, 0.050);
    if (!driveStatus.isOK()) {
      System.out.println("Could not apply drive motor configs for wheel: " + wheelID + ". Error code: " + driveStatus.toString());
      DriverStation.reportError("Could not apply drive motor configs for " + moduleName + " wheel.", false);
    } else {
      System.out.println("Successfully applied drive motor configs for wheel: " + wheelID + ". Error code: " + driveStatus.toString());
    }
    swerveDriveMotor.getConfigurator().setPosition(0);

    // Create angle motor configuration
    var angleConfigs = new TalonFXConfiguration();

    // Set angle motor output configuration
    var angleOutputConfigs = angleConfigs.MotorOutput;
    angleOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    angleOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    angleOutputConfigs.withDutyCycleNeutralDeadband(ANGLE_DEADBAND);

    // Set angle motor feedback sensor
    //var angleSensorConfig = angleConfigs.Feedback;
    //angleSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Apply angle motor configuration and initialize position to 0
    //status = StatusCode.StatusCodeNotInitialized;
    StatusCode angleStatus = swerveAngleMotor.getConfigurator().apply(angleConfigs, 0.050);
    if (!angleStatus.isOK()) {
      System.out.println("Could not apply angle motor configs for " + moduleName + " wheel." + ". Error code: " + angleStatus.toString());
      DriverStation.reportError("Could not apply angle motor configs for " + moduleName + " wheel.", false);
    } else {
      System.out.println("Successfully applied angle motor configs for wheel: " + wheelID + ". Error code: " + angleStatus.toString());
    }
    //swerveAngleMotor.getConfigurator().setPosition(0);

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
    
  }

  /**
   * 
   * Drive this wheel module at the specified speed and angle
   * 
   * @param speed  Speed for the drive motor
   * @param angle  Wheel angle for this module
   * 
   */
  public void drive(double speed, double angle) {

    SmartDashboard.putNumber(moduleName + " speed", speed);
    SmartDashboard.putNumber(moduleName + " angle", angle);

    // Normalize target to have a max value of 1
    double normAngle = angle / 360.0;
    if (normAngle == 1.0) {
      normAngle = 0.0;
    }
    SmartDashboard.putNumber(moduleName + " norm angle", normAngle);

    // double normAngle = angle;

    // Retrieve current CANcoder position. The CANcoder is configured for the range
    // [-0.5,0.5) by the
    // Phoenix Tuner X. The Phoenix 6 API returns the position in native rotation
    // units with no
    // discontinuity between 1 and 0.
    double encoderAngle = canCoder.getAbsolutePosition().getValueAsDouble();

    // Calculate distance to target angle
    double dist1 = Math.abs(normAngle - encoderAngle);
    double dist2 = 1.0 - dist1;
    double targetAngle = normAngle;
    if (dist1 > 0.25 && dist2 > 0.25) {

      targetAngle = normAngle + 0.5;
      if (targetAngle > 1.0) {
        targetAngle = targetAngle - 1.0;
      } else if (targetAngle == 1.0) {
        targetAngle = 0.0;
      }

      speed = -speed;

    }

    SmartDashboard.putNumber(moduleName + " dist1", dist1);
    SmartDashboard.putNumber(moduleName + " dist2", dist2);
    SmartDashboard.putNumber(moduleName + " target", targetAngle);

    // if (Math.max(encoderAngle - target, 1 - encoderAngle + target) > 0.25) {
    // SmartDashboard.putBoolean(moduleName + " flip", true);
    // speed = -speed;
    // target += 0.5;
    // if (target > 1)
    // target -= 1;
    // } else {
    // SmartDashboard.putBoolean(moduleName + " flip", false);
    // }

    double error = (encoderAngle - targetAngle + 0.5) % 1 - 0.5;
    SmartDashboard.putNumber(moduleName + " error", error);

    // target = encoderAngle + error;

    /*
     * //Determine shortest rotation distance
     * if (Math.abs(target - encoderAngle) > 0.5)
     * {
     * double diff = 1 - Math.abs(target - encoderAngle);
     * target = target + diff;
     * if (target >= 1.0) {
     * target = target - 1.0;
     * }
     * encoderAngle = encoderAngle + diff;
     * if (encoderAngle >= 1.0)
     * {
     * encoderAngle = encoderAngle - 1.0;
     * }
     * }
     */

    // Optimize angle movement to minimize rotational distance
    double output = wpiPIDController.calculate(error, 0);
    SmartDashboard.putNumber(moduleName + " PID output", output);

    // Calculate angle speed with limiter
    double angleSpeed = output * angleSpeedLimiter;

    // Capping angleSpeed to max that the motor can take, from -1 to 1
    if (angleSpeed > 1) {
      angleSpeed = 1;
    } else if (angleSpeed < -1) {
      angleSpeed = -1;
    }

    // Before angle speed
    SmartDashboard.putNumber(moduleName + " before angle", angleSpeed);

    // putting angleSpeed and error into smart dashboard
    SmartDashboard.putNumber(moduleName + " after angle", angleSpeed);

    // Calculate wheel velocity
    double motorVelocity = (speed / (2 * Math.PI * (WHEEL_DIAMETER / 2)) * kGearRatio);
    double motorVelocityRPM = motorVelocity * 60;
    SmartDashboard.putNumber(moduleName + " V target", motorVelocity);
    SmartDashboard.putNumber(moduleName + " target RPM", motorVelocityRPM);

    // Set motor speeds
    swerveAngleOut.Output = angleSpeed;
    swerveAngleMotor.setControl(swerveAngleOut);
    // double newSpeed = speed / LinearSpeed;
    // if (Math.abs(newSpeed) < 1.0) {
    // swerveDriveOut.Output = newSpeed;
    // } else if (newSpeed > 1.0) {
    // swerveDriveOut.Output = 1.0;
    // } else {
    // swerveDriveOut.Output = -1.0;
    // }
    // swerveDriveMotor.setControl(swerveDriveOut);
    swerveDriveMotor.setControl(m_request.withVelocity(motorVelocity));

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

    var drivePosSignal = swerveDriveMotor.getRotorPosition();
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

    var driveVelSignal = swerveDriveMotor.getRotorVelocity();
    driveVelSignal.refresh();
    return driveVelSignal.getValueAsDouble();

  }

  /**
   * 
   * Zeros the encoder for the drive motor
   * 
   */
  public void zeroEncoder() {

    //Zero twice because for some reason it doesn't want to zero sometimes
    swerveAngleOut.Output = 0.0;
    swerveAngleMotor.setControl(swerveAngleOut);
    swerveDriveOut.Output = 0.0;
    swerveDriveMotor.setControl(swerveDriveOut);

  }

  /**
   * 
   * Stop all motors
   * 
   */
  public void stop() {

    swerveDriveMotor.set(0);
    swerveAngleMotor.set(0);
  }

  /**
   * 
   * Calculate the drive distance for this module since last reset
   * 
   * @return Drive distance in meters
   * 
   */
  public double getDistance() {

    SmartDashboard.putNumber(moduleName + " distance",(WHEEL_DIAMETER * Math.PI * getDriveEncoderPosition()) / (kTalonFXPPR * DRIVE_GEAR_RATIO) );

    return (WHEEL_DIAMETER * Math.PI * getDriveEncoderPosition()) / (kTalonFXPPR * DRIVE_GEAR_RATIO);

  }

  /**
   * 
   * Calculate the drive velocity for this module
   * 
   * @return Drive velocity in meters/second
   * 
   */
  public double getWheelSpeed() {

    double rotationPerSecond = getDriveEncoderVelocity() / kTalonFXPPR * 10;
    return (WHEEL_DIAMETER * Math.PI * rotationPerSecond) / DRIVE_GEAR_RATIO;

  }

  /**
   * 
   * Get the current state of the module
   * 
   * @return Current state of this module
   * 
   */
  public SwerveModuleState getState() {

    return new SwerveModuleState(getWheelSpeed(), new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition().getValueAsDouble() * 360.0))));

  }

  /**
   * 
   * Get the current position of the module
   * 
   * @return Current position of this module
   * 
   */
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(getDistance(), new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition().getValueAsDouble() * 360.0))));

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
