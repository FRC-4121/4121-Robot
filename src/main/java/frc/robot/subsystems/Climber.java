// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * Define a climber object
 */
public class Climber extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. VAlues smaller than this will be rounded to zero

  // Declare CAN ID for motor
  private final int climberMotorID = 15;

  // Declare motor variables
  private TalonFX climberMotor;

  // Declare Phoenix PID controller gains
  private double drive_kG = 0.0;
  private double drive_kS = 0.1;
  private double drive_kV = 0.1;
  private double drive_kA = 0.0;
  private double drive_kP = 0.1;
  private double drive_kI = 0.0;
  private double drive_kD = 0.0;

  // Declare climber motor position constants
  private final int extendRotations = 1000;
  private final int retractRotations = 100;

  // Declare motor output requests
  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);

  /**
   * Create a new climber object
   */
  public Climber() {

    // Create motors
    climberMotor = new TalonFX(climberMotorID, CANBUS_NAME);

    // Create drive motor configuration
    var climberConfigs = new TalonFXConfiguration();

    // Set drive motor output configuration
    var driveOutputConfigs = climberConfigs.MotorOutput;
    driveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    driveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    driveOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set drive motor feedback sensor
    var driveSensorConfig = climberConfigs.Feedback;
    driveSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set drive motor PID constants
    var slot0Configs = climberConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply drive motor configuration and initialize position to 0
    StatusCode climberStatus = climberMotor.getConfigurator().apply(climberConfigs, 0.050);
    if (!climberStatus.isOK()) {
      System.out.println("Could not apply climber motor configs. Error code: " + climberStatus.toString());
      DriverStation.reportError("Could not apply climber motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + climberStatus.toString());
    }
    climberMotor.getConfigurator().setPosition(0);

  }

  @Override
  public void periodic() {
    
    // Put motor status on the Smart Dashboard
    SmartDashboard.putNumber("Climber Motor Amps", climberMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber Motor Volts", climberMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ClimberMotor Position", climberMotor.getPosition().getValueAsDouble());

  }

  /**
   * Extend the climber to prepare for climb
   */
  public void ExtendClimber() {

    climberMotor.setControl(m_positionRequest.withPosition(extendRotations));

  }

  /**
   * Retract the climber to climb the robot
   */
  public void RetractClimber() {

    climberMotor.setControl(m_positionRequest.withPosition(retractRotations));
    
  }

  /**
   * Get the current draw for the climber motor
   * 
   * @return  Motor amps
   */
  public double GetMotorAmps() {

    return climberMotor.getStatorCurrent().getValueAsDouble();

  }

}
