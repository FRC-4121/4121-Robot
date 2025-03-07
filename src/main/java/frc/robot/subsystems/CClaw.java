// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.Mutables;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Second;

/**
 * Define a claw (end effector) subsystem
 */
public class CClaw extends SubsystemBase {

  // Declare motor constants
  private final double DRIVE_DEADBAND = 0.001;
  private final double CURRENT_LIMIT = 65;

  // Declare CAN IDs
  private final int rotationMotorID = 19;
  private final int intakeMotorID = 20;
  private final int canRangeID = 24;

  // Declare motor variables
  private TalonFX rotationMotor;
  private TalonFX intakeMotor;
  private CANrange coralSensor;

  // Declare Phoenix PID controller gains
  private double rotate_kG = 0.2;
  private double rotate_kS = 0.1;
  private double rotate_kV = 0.1;
  private double rotate_kA = 0.0;
  private double rotate_kP = 1.0;
  private double rotate_kI = 0.0;
  private double rotate_kD = 0.0;

  // Declare motor output requests
  private final PositionVoltage requestPosition = new PositionVoltage(0.0).withSlot(0);
  private final DutyCycleOut requestRotateDuty = new DutyCycleOut(0.0);
  private final DutyCycleOut requestIntakeDuty = new DutyCycleOut(0.0);

  private static final Time extraInputTime = Time.ofBaseUnits(0.1, Second);
  private static final Time algaeIntakeTime = Time.ofBaseUnits(1.5, Second);
  private static final Time outputTime = Time.ofBaseUnits(0.25, Second);

  public static final double feedSpeed = 0.2;
  public static final double scoreSpeed = 0.25;
  public static final double algaeFeedSpeed = 0.2;
  public static final double algaeDepositSpeed = 0.1;

  // Create a claw position class
  public static final class ClawPositions {
    public static final double Load = 0;
    public static final double Home = -2.5;
    public static final double Algae = -18;
  }

  // The current position, in motor rotations
  private double currentPosition;
  // If this is true, then we want to take the current position and set it as a PID request
  private boolean holdPosition;
  // Disable the rotation safety check
  private boolean noSafety;

  private static final double minRotation = ClawPositions.Algae;
  private static final double maxRotation = ClawPositions.Home;

  /**
   * Create a claw (end effector) subsystem
   */
  public CClaw() {

    // Create motors
    rotationMotor = new TalonFX(rotationMotorID, GeneralConstants.CANBUS_NAME);
    intakeMotor = new TalonFX(intakeMotorID, GeneralConstants.CANBUS_NAME);

    // Configure the motors
    configureMotors();

    // Create CANrange
    coralSensor = new CANrange(canRangeID, GeneralConstants.CANBUS_NAME);

    // Configure CANrange
    CANrangeConfiguration sensorConfigs = new CANrangeConfiguration();
    coralSensor.getConfigurator().apply(sensorConfigs);

    // Initialize variables
    currentPosition = ClawPositions.Home;

  }

  @Override
  public void periodic() {

    // Set current position and claw clear flag
    currentPosition = getClawPosition();
    if (currentPosition < ClawPositions.Home) {
      Mutables.isClawClear = true;
    } else {
      Mutables.isClawClear = false;
      if (!noSafety) {
        setRotation(ClawPositions.Home);
      }
    }

    // Update dashboard values
    SmartDashboard.putBoolean("Claw Clear", Mutables.isClawClear);
    SmartDashboard.putNumber("Claw Rotate Amps", rotationMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Claw Rotate Volts", rotationMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Claw Rotate Pos", rotationMotor.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Claw Rotate Vel", rotationMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Has Coral", hasCoral());

    // Check motor currents and stop elevator
    if (rotationMotor.getStatorCurrent().getValueAsDouble() > CURRENT_LIMIT) {
      stopRotation();
    }

  }

  /**
   * Configure TalonFX motors
   */
  private void configureMotors() {

    // Configure the Rotation Motor
    var rotateConfigs = new TalonFXConfiguration();

    // set rotate motor output configuration
    var rotateOutputConfigs = rotateConfigs.MotorOutput;
    rotateOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rotateOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    rotateOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set rotate motor feedback sensor
    var rotateSensorConfig = rotateConfigs.Feedback;
    rotateSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set rotate motor configure limits
    var rotateLimitConfig = rotateConfigs.CurrentLimits;
    rotateLimitConfig.StatorCurrentLimitEnable = true;
    rotateLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    var motionMagicConfigs = rotateConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    motionMagicConfigs.MotionMagicAcceleration = 160;
    motionMagicConfigs.MotionMagicJerk = 1600;

    // Set rotate motor PID constants
    var Slot0Configs = rotateConfigs.Slot0;
    Slot0Configs.kG = rotate_kG;
    Slot0Configs.kS = rotate_kS;
    Slot0Configs.kV = rotate_kV;
    Slot0Configs.kA = rotate_kA;
    Slot0Configs.kP = rotate_kP;
    Slot0Configs.kI = rotate_kI;
    Slot0Configs.kD = rotate_kD;

    // Apply rotate motor configuration and initialize position to 0
    StatusCode rotationStatus = rotationMotor.getConfigurator().apply(rotateConfigs, 0.050);
    if (!rotationStatus.isOK()) {
      System.err.println("Could not apply rotation motor configs. Error code: " + rotationStatus.toString());
      DriverStation.reportError("Could not apply rotation motor configs.", false);
    } else {
      System.out.println("Successfully applied rotation motor configs. Error code: " + rotationStatus.toString());
    }

    // Configure the Intake Motor
    var intakeConfigs = new TalonFXConfiguration();

    // set intake motor output configuration
    var intakeOutputConfigs = intakeConfigs.MotorOutput;
    intakeOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set intake motor feedback sensor
    var intakeSensorConfig = intakeConfigs.Feedback;
    intakeSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set intake motor configure limits
    var intakeLimitConfig = intakeConfigs.CurrentLimits;
    intakeLimitConfig.StatorCurrentLimitEnable = true;
    intakeLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    // Apply intake motor configuration and initialize position to 0
    StatusCode intakeStatus = intakeMotor.getConfigurator().apply(intakeConfigs, 0.050);
    if (!intakeStatus.isOK()) {
      System.out.println("Could not apply intake motor configs. Error code: " + intakeStatus.toString());
      DriverStation.reportError("Could not apply intake motor configs.", false);
    } else {
      System.out.println("Successfully applied intake motor configs. Error code: " + intakeStatus.toString());
    }
    intakeMotor.getConfigurator().setPosition(0);

  }

  /**
   * 
   * Zero the position sensor for the rotation motor
   * 
   */
  public void zeroIntake() {
    rotationMotor.getConfigurator().setPosition(0);
  }

  /**
   * 
   * Rotate claw to position
   * 
   * @param rotation position in encoder units
   * 
   */
  public void setRotation(double position) {
    SmartDashboard.putBoolean("Claw Hold", false);
    holdPosition = false;
    rotationMotor.setControl(requestPosition.withPosition(position));
  }

  /**
   * 
   * Stop the rotation motor
   * 
   */
  public void stopRotation() {
    rotationMotor.stopMotor();
  }

  /**
   * Activate intake motor
   */
  public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.setControl(requestIntakeDuty.withOutput(intakeSpeed));
  }

  /**
   * Determine if we have coral onboard
   * 
   * @return Flag indicating presence of coral
   */
  public boolean hasCoral() {
    return coralSensor.getIsDetected(true).getValue();
  }

  /**
   * Command to rotate claw to home position
   * 
   * @param pos The target position
   * @return a command to rotate to the given position
   */
  public Command autoRotate(double pos) {
    return Commands.runOnce(
        () -> setRotation(pos),
        this);
  }

  /**
   * A command to stop the intake
   * 
   * @return a command that stops the intake
   */
  public Command stopIntake() {
    return Commands.runOnce(() -> this.setIntakeSpeed(0));
  }

  /**
   * Command to intake the coral
   */
  public Command intakeCoral() {
    return Commands.runOnce(() -> this.setIntakeSpeed(-feedSpeed), this).andThen(Commands.idle(this))
        .withDeadline(Commands.idle().until(this::hasCoral).andThen(Commands.waitTime(extraInputTime)))
        .andThen(stopIntake());
  }

  /**
   * Command to score coral
   */
  public Command scoreCoral() {
    return Commands.runOnce(() -> this.setIntakeSpeed(-scoreSpeed), this).andThen(Commands.idle(this))
        .withTimeout(outputTime).andThen(stopIntake());
  }

  /**
   * 
   * Rotate claw in response to gamepad joystick
   * Keep claw between home and algae positions
   * 
   * @param direction Direction and speed to rotate
   * 
   */
  public void rotate(double direction) {
    if (Math.abs(direction) < 0.0001) {
      if (holdPosition) {
        SmartDashboard.putNumber("Claw H Pos", currentPosition);
        SmartDashboard.putBoolean("Claw Hold", true);
        rotationMotor.setControl(requestPosition.withPosition(currentPosition));
        holdPosition = false;
      }
    } else {
      SmartDashboard.putBoolean("Claw Hold", false);
      holdPosition = true;
      if (!noSafety && ((currentPosition < minRotation && direction < 0) || (currentPosition > maxRotation && direction > 0))) return;
      requestRotateDuty.Output = direction;
      rotationMotor.setControl(requestRotateDuty);
    }
  }

  /**
   * 
   * Set the motor control to zero
   * 
   */
  public void killMotor() {
    holdPosition = false;
    rotationMotor.setControl(requestRotateDuty.withOutput(0));
  }

  /**
   * 
   * Get the current position of the claw in encoder units
   * 
   * @return Current claw rotation as a double
   * 
   */
  public double getClawPosition() {

    var clawPosSignal = rotationMotor.getRotorPosition();
    clawPosSignal.refresh();
    return clawPosSignal.getValueAsDouble();

  }
  
  public Command algaeIntake(){
    return Commands.runOnce(() -> this.setIntakeSpeed(algaeFeedSpeed), this).andThen(Commands.idle(this))
    .withTimeout(algaeIntakeTime).andThen(stopIntake());
  }

  public Command algaeDeposit(){
    return Commands.runOnce(() -> this.setIntakeSpeed(algaeDepositSpeed), this).andThen(Commands.idle(this))
    .withTimeout(outputTime).andThen(stopIntake());
  }

  public Command returnHome(){
    return Commands.runOnce(() -> setRotation(ClawPositions.Home));
  }

  /**
   * 
   * A command that disables the safety check to keep the motor clear of the home range.
   * 
   * This is implemented as a command so it automatically re-enables it when it's done.
   * 
   */
  public class WithoutSafety extends Command {
    @Override
    public void initialize() {
      noSafety = true;
    }
    @Override
    public void end(boolean interrupted) {
      noSafety = false;
    }
  }
}
