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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GeneralConstants;
import static frc.robot.Constants.MechanismConstants;


/**
 * 
 * Define an ElevatorMM subsystem
 * 
 */
public class ElevatorMM extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. VAlues smaller than this will be rounded
                                               // to zero
  private final double CURRENT_LIMIT = 65; // Current limit to prevent motor damage

  // Declare motor CAN IDs
  private final int elevatorLeadID = 17;
  private final int elevatorFollowID = 18;

  // Declare motor variables
  private TalonFX elevatorLeadMotor;
  private TalonFX elevatorFollowMotor;

  // Declare Phoenix PID controller gains
  private double elevator_kG = 1.0;
  private double elevator_kS = 0.25;
  private double elevator_kV = 0.1;
  private double elevator_kA = 0.0;
  private double elevator_kP = 1.0;
  private double elevator_kI = 0.0;
  private double elevator_kD = 0.0;

  // Declare elevator positions
  public static final class ElevatorPositions {
    public static final double LOAD = 2;
    public static final double CORAL1 = 10;
    public static final double CORAL2 = 10;
    public static final double CORAL3 = 57;
    public static final double CORAL4 = 120;
    public static final double ALGAE1 = 10;
    public static final double ALGAE2 = 10;
    public static final double PROCESSOR = 10;
    public static final double BARGE = 10;
  }

  // Declare motor output requests
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  // Declare other variables
  private double currentPosition;
  private boolean holdPosition;

  /**
   * 
   * Creates a new ElevatorMM object
   * 
   */
  public ElevatorMM() {

    // Create motors
    elevatorLeadMotor = new TalonFX(elevatorLeadID, GeneralConstants.CANBUS_NAME);
    elevatorFollowMotor = new TalonFX(elevatorFollowID, GeneralConstants.CANBUS_NAME);

    // Configure motors
    InitializeMotors();

    // Set follower to follow lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));

    // Initialize variables
    currentPosition = ElevatorPositions.LOAD;
    holdPosition = false;

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

    // Set current position
    currentPosition = getPosition();

    // Hold position of the elevator if requested
    // if (holdPosition && currentPosition != ElevatorPositions.LOAD) {
    //   moveElevatorToPosition(currentPosition);
    // }

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
   * @param direction Direction and speed the elevator should run
   * 
   */
  public void moveElevator(double direction) {
    if (Math.abs(direction) < 0.0001) {
      if (holdPosition) {
        SmartDashboard.putNumber("Elevator H Pos", currentPosition);
        SmartDashboard.putBoolean("Elevator Hold", true);
        elevatorLeadMotor.setControl(positionRequest.withPosition(currentPosition));
        holdPosition = false;
      }
    } else {
      SmartDashboard.putBoolean("Elevator Hold", false);
      holdPosition = true;
      elevatorLeadMotor.setControl(dutyRequest.withOutput(direction));
    }
  }

  /**
   * 
   * Run the elevator to a specified position
   * 
   * @param position The desired position of the elevator
   * 
   */
  public void moveElevatorToPosition(double position) {
    SmartDashboard.putBoolean("Elevator Hold", false);
    holdPosition = false;
    elevatorLeadMotor.setControl(positionRequest.withPosition(position));
  }

  /**
   * 
   * Gets the current elevator position
   * 
   * @return  Current position in enoder units
   */
  public double getPosition() {
    return elevatorLeadMotor.getPosition().getValueAsDouble();
  }

  /**
   * 
   * Zero the position encoder
   * 
   */
  public void zeroPosition() {
    elevatorLeadMotor.getConfigurator().setPosition(0);
  }

  /**
   * 
   * Set the position hold flag
   * 
   */
  public void setPositionHold(Boolean hold) {
    holdPosition = hold;
  }

  /**
   * 
   * Move elevator to position command
   * 
   * @param position  Desired elevator position
   * @return  The move elevator command
   * 
   */
  public Command positionElevator(double position){
    return Commands.runOnce(() -> this.moveElevatorToPosition(position));
  }
}
