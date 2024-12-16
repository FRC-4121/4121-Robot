// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private ADXRS450_Gyro gyro;
  private MedianFilter gyro_filter;

  private TalonFX leftMasterFalcon;
  private TalonFX leftSlaveFalcon;

  private TalonFX rightMasterFalcon;
  private TalonFX rightSlaveFalcon;

  private DifferentialDrive drivetrain;

  /**
   * 
   * Drivetrain constructor
   * 
   * Initialize motors, encoders, and gyro
   */
  public Drivetrain() {
    // Init motors, speed controller groups, and drivetrain
    leftMasterFalcon = new TalonFX(LEFT_MASTER_F);
    leftSlaveFalcon = new TalonFX(LEFT_SLAVE_F);

    rightMasterFalcon = new TalonFX(RIGHT_MASTER_F);
    rightSlaveFalcon = new TalonFX(RIGHT_SLAVE_F);

    drivetrain = new DifferentialDrive(leftMasterFalcon, rightMasterFalcon);

    {
      var configs = leftMasterFalcon.getConfigurator();
      configs.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive));
    }
    {
      var configs = leftSlaveFalcon.getConfigurator();
      configs.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.CounterClockwise_Positive));
    }
    {
      var configs = rightMasterFalcon.getConfigurator();
      configs.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive));
    }
    {
      var configs = rightSlaveFalcon.getConfigurator();
      configs.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive));
    }

    // Set follower mode
    leftSlaveFalcon.setControl(new Follower(LEFT_MASTER_F, false));
    rightSlaveFalcon.setControl(new Follower(RIGHT_MASTER_F, false));

    // Config encoders
    // leftMasterFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    // leftSlaveFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    // rightMasterFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    // rightSlaveFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);

    // Zero encoders
    leftMasterFalcon.setPosition(0);
    leftSlaveFalcon.setPosition(0);
    rightMasterFalcon.setPosition(0);
    rightSlaveFalcon.setPosition(0);

    SmartDashboard.putNumber("Left Drive Speed", 0);
    SmartDashboard.putNumber("Right Drive Speed", 0);

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    zeroGyro();

    // Initialize moving average filter for gyro
    // gyro_filter = LinearFilter.movingAverage(FILTER_WINDOW_SIZE);
    // gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);

    // Zero drivetrain encoders
    SmartDashboard.putNumber("Zero Encoders", 0);
    zeroEncoders();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }

  /**
   * 
   * Drivetrain periodic
   * 
   * Things that need to happen on a periodic basis as
   * the drivetrain is being used.
   * 
   */
  @Override
  public void periodic() {

    // Put info on the dashboard
    // SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("Left Master Encoder", leftMasterFalcon.getPosition().refresh().getValue());
    SmartDashboard.putNumber("Right Master Encoder", rightMasterFalcon.getPosition().refresh().getValue());
    SmartDashboard.putNumber("Left Master Velocity", leftMasterFalcon.getVelocity().refresh().getValue());
    SmartDashboard.putNumber("Right Master Velocity", rightMasterFalcon.getVelocity().refresh().getValue());

    // Zero the gyro on driver command
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro == 1) {
      SmartDashboard.putNumber("Zero Gyro", 0);
      gyro.calibrate();
      zeroGyro();
    }

    // Zero the encoders on driver command
    double zeroEncoders = SmartDashboard.getNumber("Zero Encoders", 0);
    if (zeroEncoders == 1) {
      SmartDashboard.putNumber("Zero Encoders", 0);
      gyro.calibrate();
      zeroEncoders();
    }

  }

  /**
   * 
   * Main teleop drive method
   * 
   * @param leftJoyY:  left joystick position
   * @param rightJoyY: right joystick position
   * 
   */
  public void drive(double leftJoyY, double rightJoyY) {

    // Drive the motors
    // Direction multiplier indicates drive direction
    SmartDashboard.putNumber("Invert DRive", DIRECTION_MULTIPLIER);
    if (DIRECTION_MULTIPLIER == 1) {
      SmartDashboard.putNumber("Left Drive Speed", currentGear * DIRECTION_MULTIPLIER * leftJoyY);
      SmartDashboard.putNumber("Right Drive Speed", currentGear * DIRECTION_MULTIPLIER * rightJoyY);
      drivetrain.tankDrive(currentGear * DIRECTION_MULTIPLIER * leftJoyY * kSpeedCorrection,
          currentGear * DIRECTION_MULTIPLIER * rightJoyY);
    } else {
      SmartDashboard.putNumber("Left Drive Speed", currentGear * DIRECTION_MULTIPLIER * rightJoyY);
      SmartDashboard.putNumber("Right Drive Speed", currentGear * DIRECTION_MULTIPLIER * leftJoyY);
      drivetrain.tankDrive(currentGear * DIRECTION_MULTIPLIER * rightJoyY,
          currentGear * DIRECTION_MULTIPLIER * leftJoyY * kSpeedCorrection);
    }

    // SmartDashboard.putNumber("Left Master Voltage",
    // leftMasterFalcon.getOutputVoltage());
    // SmartDashboard.putNumber("Right Master Voltage",
    // rightMasterFalcon.getOutputVoltage());

  }

  /**
   * 
   * Run drivetrain during autonomous
   * 
   * @param leftSpeed:  speed for left side motors
   * @param rightSpeed: speed for right side motors
   * 
   */
  public void autoDrive(double leftSpeed, double rightSpeed) {
    SmartDashboard.putNumber("Left Drive Speed", leftSpeed);
    SmartDashboard.putNumber("Right Drive Speed", rightSpeed);
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  /** Stop the drive train */
  public void stopDrive() {

    drivetrain.tankDrive(0, 0);
  }

  /** Zero the encoders */
  public void zeroEncoders() {

    leftMasterFalcon.setPosition(0);
    leftSlaveFalcon.setPosition(0);
    rightMasterFalcon.setPosition(0);
    rightSlaveFalcon.setPosition(0);

  }

  // /**
  // *
  // * Get position of all left encoders
  // *
  // * @return array of encoder positions
  // *
  // */
  // public double[] getLeftEncoders() {

  // double[] encoders = new double[2];

  // encoders[0] = leftMasterFalcon.getPosition().refresh().getValue();
  // encoders[1] = leftSlaveFalcon.getPosition().refresh().getValue();

  // return encoders;

  // }

  // /**
  // *
  // * Get position of all right encoders
  // *
  // * @return array of encoder positions
  // *
  // */
  // public double[] getRightEncoders() {

  // double[] encoders = new double[2];

  // encoders[0] = rightMasterFalcon.getPosition().refresh().getValue();
  // encoders[1] = rightSlaveFalcon.getPosition().refresh().getValue();

  // return encoders;

  // }

  // /**
  // *
  // * Get position of left master encoder
  // *
  // * @return encoder position
  // *
  // */
  // public double getMasterLeftEncoderPosition() {

  // return leftMasterFalcon.getSelectedSensorPosition();

  // }

  // /**
  // *
  // * Get position of right master encoder
  // *
  // * @return encoder position
  // *
  // */
  // public double getMasterRightEncoderPosition() {

  // return rightMasterFalcon.getSelectedSensorPosition();

  // }

  /**
   * 
   * Reset current gyro heading to zero
   * 
   */
  public void zeroGyro() {
    // gyro.calibrate();
    gyro.reset();

  }

  /** Invert the direction of driving */
  public void invertDirection() {

    DIRECTION_MULTIPLIER *= -1;

  }

  public void changeGears() {

    if (currentGear == kLowGearMultiplier) {
      currentGear = kHighGearMultiplier;
    } else {

      currentGear = kLowGearMultiplier;
    }
  }

  public double getGyroAngle() {

    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    return correctedGyro;

  }

}
