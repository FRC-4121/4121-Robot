// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants;

import java.util.HashMap;
import java.util.Map;

/**
 * 
 * Define an ElevatorMM subsystem
 * 
 */
public class ElevatorMM extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. VAlues smaller than this will be rounded
                                               // to zero
  private final double CURRENT_LIMIT = 100; // Current limit to prevent motor damage

  // Declare motor CAN IDs
  private final int elevatorLeadID = 13;
  private final int elevatorFollowID = 14;

  private final double elevatorSpeed = 0.5;

  // Declare motor variables
  private TalonFX elevatorLeadMotor;
  private TalonFX elevatorFollowMotor;

  // Declare Phoenix PID controller gains
  private double elevator_kG = 0.0;
  private double elevator_kS = 0.1;
  private double elevator_kV = 0.1;
  private double elevator_kA = 0.0;
  private double elevator_kP = 0.1;
  private double elevator_kI = 0.0;
  private double elevator_kD = 0.0;

  public static final class Positions {
    public static final double LOAD = 100;
    public static final double CORAL1 = 100;
    public static final double CORAL2 = 100;
    public static final double CORAL3 = 100;
    public static final double CORAL4 = 100;
    public static final double ALGAE1 = 100;
    public static final double ALGAE2 = 100;
    public static final double PROCESSOR = 100;
    public static final double BARGE = 100;
  }

  // Declare motor output requests
  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut m_dutyRequest = new DutyCycleOut(0);

  /**
   * 
   * Creates a new ElevatorMM object
   * 
   */
  public ElevatorMM() {

    // Create motors
    elevatorLeadMotor = new TalonFX(elevatorLeadID, CANBUS_NAME);
    elevatorFollowMotor = new TalonFX(elevatorFollowID, CANBUS_NAME);

    // Configure motors
    InitializeMotors();

    // Set follower to follow lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));

  }

  /**
   * 
   * Configure the motors
   * 
   */
  private void InitializeMotors() {

    // Configure the lead elevator motor
    var leadConfigs = new TalonFXConfiguration();

    // Set lead motor output configuration
    var leadOutputConfigs = leadConfigs.MotorOutput;
    leadOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leadOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    leadOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set lead motor feedback sensor
    var leadSensorConfig = leadConfigs.Feedback;
    leadSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set lead motor current limits
    var leadLimitConfig = leadConfigs.CurrentLimits;
    leadLimitConfig.StatorCurrentLimitEnable = true;
    leadLimitConfig.StatorCurrentLimit = 110;

    // Set drive motor PID constants
    var slot0Configs = leadConfigs.Slot0;
    slot0Configs.kG = elevator_kG;
    slot0Configs.kS = elevator_kS;
    slot0Configs.kV = elevator_kV;
    slot0Configs.kA = elevator_kA;
    slot0Configs.kP = elevator_kP;
    slot0Configs.kI = elevator_kI;
    slot0Configs.kD = elevator_kD;

    // Set MotionMagic settings
    var motionMagicConfigs = leadConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    // Apply lead motor configuration and initialize position to 0
    StatusCode leadStatus = elevatorLeadMotor.getConfigurator().apply(leadConfigs, 0.050);
    if (!leadStatus.isOK()) {
      System.out.println("Could not apply lead elevator motor configs. Error code: " + leadStatus.toString());
      DriverStation.reportError("Could not apply lead elevator motor configs.", false);
    } else {
      System.out.println("Successfully applied lead elevator motor configs. Error code: " + leadStatus.toString());
    }
    elevatorLeadMotor.getConfigurator().setPosition(0);

    // Configure the follower elevator motor
    var followConfigs = new TalonFXConfiguration();

    // Set follower motor output configuration
    var followOutputConfigs = followConfigs.MotorOutput;
    followOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    followOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    followOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set follower motor current limits
    var followLimitConfig = followConfigs.CurrentLimits;
    followLimitConfig.StatorCurrentLimitEnable = true;
    followLimitConfig.StatorCurrentLimit = 110;

    // Apply follow motor configuration
    StatusCode followStatus = elevatorFollowMotor.getConfigurator().apply(followConfigs, 0.050);
    if (!followStatus.isOK()) {
      System.out.println("Could not apply follower elevator motor configs. Error code: " + followStatus.toString());
      DriverStation.reportError("Could not apply follower elevator motor configs.", false);
    } else {
      System.out.println("Successfully applied follower elevator motor configs. Error code: " + leadStatus.toString());
    }

  }

  /**
   * 
   * Regular output of critical data
   * 
   */
  @Override
  public void periodic() {

    // Put critical lead motor signals on the SmartDashboard
    SmartDashboard.putNumber("Elevator Lead Amps", elevatorLeadMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Lead Volts", elevatorLeadMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Lead Pos", elevatorLeadMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Lead Vel", elevatorLeadMotor.getVelocity().getValueAsDouble());

    // Put critical follower motor signals on the SmartDashboard
    SmartDashboard.putNumber("Elevator Follow Amps", elevatorFollowMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follow Volts", elevatorFollowMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follow Pos", elevatorFollowMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follow Vel", elevatorFollowMotor.getVelocity().getValueAsDouble());

    // Check motor currents and stop elevator
    if (elevatorLeadMotor.getStatorCurrent().getValueAsDouble() > CURRENT_LIMIT ||
        elevatorFollowMotor.getStatorCurrent().getValueAsDouble() > CURRENT_LIMIT) {
      stopElevator();
    }

  }

  /**
   * 
   * Stop the elevator
   * 
   */
  public void stopElevator() {
    elevatorLeadMotor.stopMotor();
  }

  /**
   * 
   * Run the elevator in response to operator input
   * 
   * @param direction Direction the elevator should run
   * 
   */
  public void moveElevator(double direction) {
    elevatorLeadMotor.setControl(m_dutyRequest.withOutput(direction * elevatorSpeed));
  }

  /**
   * 
   * Run the elevator to a specified position
   * 
   * @param position The desired position of the elevator
   * 
   */
  public void moveElevatorToPosition(double position) {
    elevatorLeadMotor.setControl(m_positionRequest.withPosition(position));
  }
}
