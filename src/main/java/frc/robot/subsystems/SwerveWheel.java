// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.ExtraClasses.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoder.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveWheel extends SubsystemBase { 

  // Declare motor variables
  private WPI_TalonFX swerveDriveMotor;
  private WPI_TalonFX swerveAngleMotor;
  private CANCoder canCoder;

  // Declare controller variables
  private PIDControl anglePIDController;
  private PIDController wpiPIDController;
  
  private double angleSpeedLimiter;
  private double kP_AngleController;
  private double kI_AngleController;
  private double kD_AngleController;

  // Declare state variables
  private double wheelSpeed;
  private double wheelAngle;
  private int wheelID;

  // Declare filters
  private MedianFilter angle_filter;
  private SlewRateLimiter rateLimiter;

  // Declare general variables
  private String moduleName;
 

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
  public SwerveWheel(String name, int driveMotorID, int angleMotorID, int CANCoderID) {
    
    // Set variables
    moduleName = name;

    // Set wheel ID
    wheelID = CANCoderID / 3;

    // Set controller constants
    angleSpeedLimiter = angleLimiters[wheelID - 1];
    kP_AngleController = anglePIDkPs[wheelID - 1];
    kI_AngleController = anglePIDkIs[wheelID - 1];
    kD_AngleController = anglePIDkDs[wheelID - 1];

    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID, CANCoderID);

    // Initialize PID controller
    anglePIDController = new PIDControl(kP_AngleController, kI_AngleController, kD_AngleController);
    wpiPIDController = new PIDController(kP_AngleController, kI_AngleController, kD_AngleController);
    wpiPIDController.setTolerance(1.5,5);

    // Create filters
    angle_filter = new MedianFilter(FILTER_WINDOW_SIZE);
    rateLimiter = new SlewRateLimiter(100000.0, -50.0, 0.0);

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

    // Create motors
    swerveDriveMotor = new WPI_TalonFX(driveMotorID);
    swerveAngleMotor = new WPI_TalonFX(angleMotorID);
    canCoder = new CANCoder(CANCoderID);

    // Reset motors to factory default settings
    swerveDriveMotor.configFactoryDefault();
    swerveAngleMotor.configFactoryDefault();

    // Set brake mode
    swerveDriveMotor.setNeutralMode(NeutralMode.Brake);
    swerveAngleMotor.setNeutralMode(NeutralMode.Brake);

    // Set motor inversions
    swerveDriveMotor.setInverted(!kMotorInvert);
    swerveAngleMotor.setInverted(kMotorInvert);

    // Set motor encoders
    swerveDriveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    swerveAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    // swerveDriveMotor.config_kP(0, drivePIDkPs[wheelID - 1], kTimeoutMsDrive);
    // swerveDriveMotor.config_kI(0, drivePIDkIs[wheelID - 1], kTimeoutMsDrive);
    // swerveDriveMotor.config_kD(0, drivePIDkDs[wheelID - 1], kTimeoutMsDrive);
    swerveDriveMotor.config_kF(0, drivePIDkFs[wheelID - 1], kTimeoutMsDrive);

    // Set motor deadband
    swerveDriveMotor.configNeutralDeadband(kDriveDeadband, kTimeoutMsDrive);
    swerveAngleMotor.configNeutralDeadband(kAngleDeadband, kTimeoutMsDrive);

    // Set frame periods
    swerveDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMsDrive);
    swerveDriveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMsDrive);

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

    SmartDashboard.putNumber(moduleName + " angle", angle);

    // Normalize target to have a max value of 1
    double target = angle / 360.0;
    if (target == 1.0) {
      target = 0.0;
    }
    //double target = angle;

    // Normalize encoder to have a max value of 1 and correct for discontinuity at 360 degrees (should be 0)
    double encoderAngle = canCoder.getAbsolutePosition() / 360;
    if (encoderAngle == 1.0) {
      encoderAngle = 0.0;
    }
    //double encoderAngle = canCoder.getAbsolutePosition() / 180;

    //Putting CANCoder position on smart dashboard
    SmartDashboard.putNumber("Wheel "+ wheelID, encoderAngle);
  
    double dist1 = Math.abs(target-encoderAngle);
    double dist2 = 1.0 - dist1;

    if (dist1 > 0.25 && dist2 > 0.25) {

      target = target + 0.5;
      if (target > 1.0) {
        target = target - 1.0;
      } else if (target == 1.0) {
        target = 0.0;
      }

      speed = -speed;
    
    }
    
    /*//Determine shortest rotation distance
    if (Math.abs(target - encoderAngle) > 0.5)
    {
      double diff = 1 - Math.abs(target - encoderAngle);
      target = target + diff;
      if (target >= 1.0) {
        target = target - 1.0;
      }
      encoderAngle = encoderAngle + diff;
      if (encoderAngle >= 1.0)
      {
        encoderAngle = encoderAngle - 1.0;
      }
    }*/

    // Optimize angle movement to minimize rotational distance
    
    
    //double output = anglePIDController.run(encoderAngle,target);
    double output = wpiPIDController.calculate(encoderAngle,target);
    SmartDashboard.putNumber("Wheel " + wheelID + " PID output", output);
    SmartDashboard.putNumber("Wheel " + wheelID + " Target", target);
    SmartDashboard.putNumber("Wheel " + wheelID + " Corrected Encoder", encoderAngle);
    
    double angleSpeed = output * angleSpeedLimiter;
    
    //Capping angleSpeed to max that the motor can take, from -1 to 1
    if (angleSpeed > 1)
    {
      angleSpeed = 1;
    } else if (angleSpeed < -1)
    {
      angleSpeed = -1;
    }

    //Before angle speed
    SmartDashboard.putNumber("Wheel " + wheelID + " before angle", angleSpeed);

    //Get the sign of the angleSpeed
    double sign = Math.signum(angleSpeed);

    //Enforce minimum angle speed, 0.0028 is one degree
    // double angleDiff = target - encoderAngle;
    // double angleError = Math.abs(angleDiff);
    // if (angleError < 0.01)
    // {
    //   if (angleError > 0.001)
    //   {
    //     angleSpeed = 0.06 * sign;
    //   }
    //   else
    //   {
    //     angleSpeed = 0.0;
    //   }
    // } 
    // else 
    // {
    //   angleSpeed = 0;
    // }

    //putting angleSpeed and error into smart dashboard
    SmartDashboard.putNumber("Wheel " + wheelID + " after angle", angleSpeed);
    SmartDashboard.putNumber("Wheel " + wheelID + " error", encoderAngle - target);
    
    //Calculate wheel velocity
    double wheelVelocity = 23712 * (speed * swerveDriveSpeedLimiter) - 894.29;
    SmartDashboard.putNumber("Wheel " + wheelID + " V target", wheelVelocity);

    //Set motor speeds
    swerveAngleMotor.set(angleSpeed);
    swerveDriveMotor.set(ControlMode.Velocity, wheelVelocity);
    SmartDashboard.putNumber("Wheel " + wheelID + " V actual", swerveDriveMotor.getSelectedSensorVelocity());
    
  }


  /**
   *
   * Get the encoder value of the drive motor
   * 
   * @return Current position (number of pulses) of the drive encoder
   * 
   */
  public double getDriveEncoderPosition() {

    return swerveDriveMotor.getSelectedSensorPosition();

  }

  /**
   * 
   * Zeros the encoder for the drive motor
   * 
   */
  public void zeroEncoder() {

    //Zero twice because for some reason it doesn't want to zero sometimes
    swerveDriveMotor.setSelectedSensorPosition(0);
    swerveDriveMotor.setSelectedSensorPosition(0);

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

    SmartDashboard.putNumber(moduleName + " distance",(kWheelDiameter * Math.PI * swerveDriveMotor.getSelectedSensorPosition()) / (kTalonFXPPR * kDriveGearRatio) );

    return (kWheelDiameter * Math.PI * swerveDriveMotor.getSelectedSensorPosition()) / (kTalonFXPPR * kDriveGearRatio);

  }

  /**
   * 
   * Calculate the drive velocity for this module
   * 
   * @return Drive velocity in meters/second
   * 
   */
  public double getWheelSpeed() {

    double rotationPerSecond = (double)swerveDriveMotor.getSelectedSensorVelocity() / kTalonFXPPR * 10;
    return (kWheelDiameter * Math.PI * rotationPerSecond) / kDriveGearRatio;

  }

  /**
   * 
   * Get the current state of the module
   * 
   * @return Current state of this module
   * 
   */
  public SwerveModuleState getState() {

    return new SwerveModuleState(getWheelSpeed(), new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition()))));

  }

  /**
   * 
   * Get the current position of the module
   * 
   * @return Current position of this module
   * 
   */
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(getDistance(), new Rotation2d(Math.toRadians(toWPIAngle(canCoder.getAbsolutePosition()))));

  }

   public double fromWPIAngle(double angle) {
    if (angle > 0) {
      angle = (180 - angle) + 180;
    } else if (angle < 0) {
      angle = -angle;
    }
    return angle;
  }

  public double toWPIAngle(double angle) {
    if (angle > 180) {
      angle = -(angle - 360);
    } else if (angle > 0 && angle <=180) {
      angle = -angle;
    }
    return angle;
  }
  
}
