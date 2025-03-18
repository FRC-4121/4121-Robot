// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.Mutables;
import frc.robot.commands.TimeoutCommand;

/**
 * 
 * Define an ElevatorMM subsystem
 * 
 */
public class ElevatorMM extends SubsystemBase {

  // Declare constants
  private static final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. VAlues smaller than this will be rounded
                                               // to zero
  private static final double CURRENT_LIMIT = 20; // Current limit to prevent motor damage

  // Declare motor CAN IDs
  private static final int elevatorLeadID = 17;
  private static final int elevatorFollowID = 18;
  private static final int limitSwitchID = 1;

  // Declare motor variables
  private final TalonFX elevatorLeadMotor;
  private final TalonFX elevatorFollowMotor;
  private final DigitalInput limitSwitch;

  // Declare Phoenix PID controller gains
  private static final Slot0Configs autoGains = new Slot0Configs() {
    {
      kG = 1.0;
      kS = 0.25;
      kV = 0.1;
      kP = 1.0;
      kI = 0.1;
      kD = 0.0;
    }
  };

  // Declare elevator positions
  public static final class ElevatorPositions {
    public static final double Load = 0;
    public static final double Coral1 = 1;
    public static final double Coral2 = 16;
    public static final double Coral3 = 55;
    public static final double Coral4 = 119;
    public static final double Algae1 = 31;
    public static final double Algae2 = 80;
    public static final double Processor = 10;
    public static final double Barge = 10;
  }

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
    limitSwitch = new DigitalInput(limitSwitchID);

    // Configure motors
    InitializeMotors();

    // Set follower to follow lead motor
    elevatorFollowMotor.setControl(new Follower(elevatorLeadMotor.getDeviceID(), false));

    // Initialize variables
    currentPosition = ElevatorPositions.Load;
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
    leadLimitConfig.StatorCurrentLimit = 100;

    // Set drive motor PID constants
    leadConfigs.Slot0 = autoGains;

    // Set MotionMagic settings
    var motionMagicConfigs = leadConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 70;
    motionMagicConfigs.MotionMagicAcceleration = 85;
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
    followLimitConfig.StatorCurrentLimit = 100;

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

    if (limitSwitch.get()) {
      zeroPosition();
    }

    // Set current position
    currentPosition = getPosition();

    SmartDashboard.putBoolean("Elevator Limit Switch", limitSwitch.get());

    // Hold position of the elevator if requested
    // if (holdPosition && currentPosition != ElevatorPositions.LOAD) {
    // moveElevatorToPosition(currentPosition);
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
    if (Math.abs(elevatorLeadMotor.getStatorCurrent().getValueAsDouble()) > CURRENT_LIMIT ||
        Math.abs(elevatorFollowMotor.getStatorCurrent().getValueAsDouble()) > CURRENT_LIMIT) {
      stopElevator();
      // zeroPosition();
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
          elevatorLeadMotor.setControl(new PositionVoltage(currentPosition).withSlot(0));
          holdPosition = false;
      }
    } else {
      SmartDashboard.putBoolean("Elevator Hold", false);
      holdPosition = true;
      elevatorLeadMotor.setControl(new DutyCycleOut(direction));
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
    elevatorLeadMotor.setControl(new PositionVoltage(position));
  }

  /**
   * 
   * Gets the current elevator position
   * 
   * @return Current position in enoder units
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
  public void setPositionHold(boolean hold) {
    holdPosition = hold;
  }

  /**
   * 
   * Set the motor control to zero
   * 
   */
  public void killMotor() {
    holdPosition = false;
    elevatorLeadMotor.setControl(new DutyCycleOut(0));
  }

  /**
   * 
   * Move elevator to position command
   * 
   * @param position Desired elevator position
   * @return The move elevator command
   * 
   */
  public Command positionElevator(double position) {
    return Commands.runOnce(() -> moveElevatorToPosition(position));
  }

  
  private SubsystemBase getThis() {
    return this;
  }

  /**
   * 
   * This command tells the elevator to move to a position, but isn't finished until it actually gets there
   * 
   */
  public class PositionElevatorAndWait extends TimeoutCommand {
    private double position;

    public PositionElevatorAndWait(double position) {
      super(1.0);
      this.position = position;
      addRequirements(getThis());
    }

    @Override
    public void initialize() {
      moveElevatorToPosition(position);
    }

    @Override
    public boolean isFinished() {
      if (super.isFinished()) return true;
      double err = Math.abs(currentPosition - position);
      return err < 1.5;
    }
  }
}
