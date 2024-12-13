// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.ExtraClasses.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveWheel extends SubsystemBase { 

  // Hardware variables
  private TalonFX swerveDriveMotor;
  private TalonFX swerveAngleMotor;
  private CANcoder canCoder;

  // PID controllers. May be removed (needs testing).
  private PIDControl anglePIDController;
  private PIDController wpiPIDController;

  private WheelConfig config;

  // Static configs.
  private static final double configTimeout = 0.01; // Timeout for each configuration, in seconds.
  private static final double angleDeadband = 0.001; // Deadband for the angle motor. Values smaller than this are rounded to 0.
  private static final double driveDeadband = 0.001; // Deadband for the drive motor. This works in the same way as the angle one.
  private static final double wheelDiameter = 0.1016;
  private static final double driveGearRatio = 8.14;

  // Configuration for the swerve wheel.
  public static class WheelConfig {
    // The name of the wheel, to be used for logging.
    public String name = "";
    // The CAN ID for the drive motor. -1 is used as a placeholder.
    public int driveId = -1;
    // The CAN ID for the angle motor. -1 is used as a placeholder.
    public int angleId = -1;
    // The CAN ID for the CANcoder. -1 is used as a placeholder.
    public int coderId = -1;
    // Limit on the angle speed. Ranges from 0 to 1 I think?
    public double angleLimiter = 1.0;
    // kP for the PID controller for the angle motor.
    public double angleKP = 0.0;
    // kI for the PID controller for the angle motor.
    public double angleKI = 0.0;
    // kD for the PID controller for the angle motor.
    public double angleKD = 0.0;
    // kF for the drive motor controller. Might be kV now?
    public double driveKF = 1.0;
    public WheelConfig() {}
    public WheelConfig(String name) {
      this.name = name;
    }
    public WheelConfig(String name, int driveId, int angleId) {
      this.name = name;
      this.driveId = driveId;
      this.angleId = angleId;
    }
    public WheelConfig withName(String name) {
      this.name = name;
      return this;
    }
    public WheelConfig withDriveId(int driveId) {
      this.driveId = driveId;
      return this;
    }
    public WheelConfig withAngleId(int angleId) {
      this.angleId = angleId;
      return this;
    }
    public WheelConfig withCoderId(int coderId) {
      this.coderId = coderId;
      return this;
    }
    public WheelConfig withIds(int driveId, int angleId) {
      this.driveId = driveId;
      this.angleId = angleId;
      return this;
    }
    public WheelConfig withIds(int driveId, int angleId, int coderId) {
      this.driveId = driveId;
      this.angleId = angleId;
      this.coderId = coderId;
      return this;
    }
    public WheelConfig withAngleLimiter(int speedLimit) {
      this.angleLimiter = speedLimit;
      return this;
    }
    public WheelConfig clone() {
      try {
        return (WheelConfig)super.clone();
      } catch (CloneNotSupportedException ex) {
        throw new RuntimeException(ex); // this path should never be taken but Java makes me
      }
    }
  }

  public static final WheelConfig baseConfig = new WheelConfig() {
    {
      angleKP = 3.25;
      angleKI = 2.25;
      angleKP = 0.04;
      driveKF = 0.0454;
    }
  };

  /**
   * 
   *  Creates a new SwerveWheel
   * 
   * @param config Configuration to be used for the wheel.
   */
  public SwerveWheel(WheelConfig config) {
    this.config = config;

    // Initialize the drive motor
    if (config.driveId < 0) {
      throw new IllegalArgumentException("Wheel config has a drive motor ID less than 0");
    }
    swerveDriveMotor = new TalonFX(config.driveId);
    var driveConfig = swerveDriveMotor.getConfigurator(); // The configurator is used for more advanced configurations
    {
      StatusCode code = driveConfig.apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor), configTimeout);
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring feedback for drive failed with code: %s", code), false);
      }
    }
    {
      StatusCode code = driveConfig.apply(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive) // Clockwise positive for this motor.
          .withNeutralMode(NeutralModeValue.Brake) // When we aren't writing a value to the motor, we want to brake.
          .withDutyCycleNeutralDeadband(driveDeadband), // Values smaller than the drive deadband are rounded down to 0.
        configTimeout
      );
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring output levels for drive motor failed with code: %s", code), false);
      }
    }

    // Initialize the angle motor
    if (config.angleId < 0) {
      throw new IllegalArgumentException("Wheel config has a angle motor ID less than 0");
    }
    swerveAngleMotor = new TalonFX(config.angleId);
    var angleConfig = swerveAngleMotor.getConfigurator(); // The configurator is used for more advanced configurations
    {
      StatusCode code = angleConfig.apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor), configTimeout);
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring feedback for angle motor failed with code: %s", code), false);
      }
    }
    {
      StatusCode code = angleConfig.apply(
        new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive) // Clockwise positive for this motor.
          .withNeutralMode(NeutralModeValue.Brake) // When we aren't writing a value to the motor, we want to brake.
          .withDutyCycleNeutralDeadband(angleDeadband), // Values smaller than the drive deadband are rounded down to 0.
        configTimeout
      );
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring output levels for angle motor failed with code: %s", code), false);
      }
    }

    if (config.coderId >= 0) {
      canCoder = new CANcoder(config.coderId); // Only initialize the CANcoder if it isn't a placeholder.
    }

    // We have two PID controllers, we should probably figure out what they do and if we need them
    anglePIDController = new PIDControl(config.angleKP, config.angleKI, config.angleKD);
    wpiPIDController = new PIDController(config.angleKP, config.angleKI, config.angleKD);
    wpiPIDController.setTolerance(1.5,5);

    // these were in the old code, I'm not sure what they do or how to replace them.
    // swerveDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMsDrive);
    // swerveDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMsDrive);
  }

  /**
   * 
   * Periodically set wheel speed and angle
   * 
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
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

    SmartDashboard.putNumber(config.name + " angle", angle);

    // Normalize target to have a max value of 1
    double target = angle / 360.0;
    if (target == 1.0) {
      target = 0.0;
    }
    double wheelVelocity = 23712 * (speed * swerveDriveSpeedLimiter) - 894.29;
    SmartDashboard.putNumber(config.name + " V target", wheelVelocity);

    if (canCoder != null) { // old code that uses an external CANcoder
      // Normalize encoder to have a max value of 1 and correct for discontinuity at 360 degrees (should be 0)
      double encoderAngle = canCoder.getAbsolutePosition().getValue();
      if (encoderAngle == 1.0) {
        encoderAngle = 0.0;
      }

      SmartDashboard.putNumber(config.name + " encoder angle", encoderAngle);
    
      double dist1 = Math.abs(target - encoderAngle);
      double dist2 = 1.0 - dist1;

      // Normalize the target angle for the PID controller
      if (dist1 > 0.25 && dist2 > 0.25) {
        target += 0.5;
        if (target > 1.0) {
          target -= 1.0;
        } else if (target == 1.0) {
          target = 0.0;
        }
        speed = -speed;
      }
      
      // double output = anglePIDController.run(encoderAngle, target);
      double output = wpiPIDController.calculate(encoderAngle, target);
      SmartDashboard.putNumber(config.name + " PID output", output);
      SmartDashboard.putNumber(config.name + " Target", target);
      SmartDashboard.putNumber(config.name + " Corrected Encoder", encoderAngle);
      
      double angleSpeed = output * config.angleLimiter;
      
      // Cap the angleSpeed to max that the motor can take, from -1 to 1
      if (angleSpeed > 1) {
        angleSpeed = 1;
      } else if (angleSpeed < -1) {
        angleSpeed = -1;
      }

      //Before angle speed
      SmartDashboard.putNumber(config.name + " before angle", angleSpeed);
      
      //putting angleSpeed and error into smart dashboard
      SmartDashboard.putNumber(config.name + " after angle", angleSpeed);
      SmartDashboard.putNumber(config.name + " error", encoderAngle - target);
      
      //Calculate wheel velocity
      swerveAngleMotor.set(angleSpeed);
      swerveDriveMotor.set(wheelVelocity);
    } else { // this looks a lot simpler but is untested
      swerveAngleMotor.setControl(new PositionVoltage(angle));
      swerveDriveMotor.set(wheelVelocity);
    }

    //Set motor speeds
    SmartDashboard.putNumber(config.name + " V actual", swerveDriveMotor.getVelocity().refresh().getValue());
  }

  /**
   * 
   * Zero the encoder for the drive motor
   * 
   */
  public void zeroEncoder() {
    //Zero twice because for some reason it doesn't want to zero sometimes
    swerveDriveMotor.setPosition(0, configTimeout);
    swerveDriveMotor.setPosition(0, configTimeout);
  }

  /**
   * 
   * Stop all motors
   * 
   */
  public void stop() {
    swerveAngleMotor.setControl(new StaticBrake());
    swerveDriveMotor.setControl(new StaticBrake());
  }

  /**
   * 
   * Calculate the drive distance for this module since last reset
   * 
   * @return Drive distance in meters
   * 
   */
  public double getDistance() {
    double dist = (wheelDiameter * Math.PI * swerveDriveMotor.getPosition().refresh().getValue()) / (kTalonFXPPR * driveGearRatio);
    SmartDashboard.putNumber(config.name + " distance", dist);
    return dist;
  }

  /**
   * 
   * Calculate the drive velocity for this module
   * 
   * @return Drive velocity in meters/second
   * 
   */
  public double getWheelSpeed() {
    double rotationPerSecond = (double)swerveDriveMotor.getVelocity().refresh().getValue() / kTalonFXPPR * 10;
    return (wheelDiameter * Math.PI * rotationPerSecond) / driveGearRatio;
  }

  /**
   * 
   * Get the current state of the module
   * 
   * @return Current state of this module
   * 
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getWheelSpeed(),
      new Rotation2d(Math.toRadians(Utils.toWPIAngle(canCoder.getAbsolutePosition().refresh().getValue())))
    );
  }

  /**
   * 
   * Get the current position of the module
   * 
   * @return Current position of this module
   * 
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDistance(),
      new Rotation2d(Math.toRadians(Utils.toWPIAngle(canCoder.getAbsolutePosition().refresh().getValue())))
    );
  }
}
