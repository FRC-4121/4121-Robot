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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants.GeneralConstants;

/**
 * Define a climber object
 */
public class Climber extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. VAlues smaller than this will be rounded
                                               // to zero
  private final double CURRENT_LIMIT = 100; // Current limit for stopping motor to prevent damage

  // Declare CAN ID for motor
  private final int climberMotorID = 25;

  //Declare PWM ID for servos
  private final int brakeServoID = 0;
  private final int rampServo1ID = 1;
  private final int rampServo2ID = 2;

  // Declare Kracken motor variables
  private TalonFX climberMotor;

  // Declare servo motor variables
  private Servo brakeServo;
  private Servo rampServo1;
  private Servo rampServo2;

  // Declare Phoenix PID controller gains
  private double drive_kG = 0.0;
  private double drive_kS = 0.1;
  private double drive_kV = 0.1;
  private double drive_kA = 0.0;
  private double drive_kP = 5.0;
  private double drive_kI = 0.0;
  private double drive_kD = 0.0;

  private boolean holdPosition;
  private double currentPosition;

  // Declare climber motor position constants
  public static final class ClimberPositions {
    public static final int Extend = -285;
    public static final int Retract = -47;
    public static final int Home = 0;
  }

  /**
   * Create a new climber object
   */
  public Climber() {

    // Create motors
    climberMotor = new TalonFX(climberMotorID, GeneralConstants.CANBUS_NAME);

    // Create climber motor configuration
    var climberConfigs = new TalonFXConfiguration();

    // Set climber motor output configuration
    var climberOutputConfigs = climberConfigs.MotorOutput;
    climberOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    climberOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set climber current limits configuration
    var climberLimitConfig = climberConfigs.CurrentLimits;
    climberLimitConfig.StatorCurrentLimitEnable = true;
    climberLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    // Set climber motor feedback sensor
    var climberSensorConfig = climberConfigs.Feedback;
    climberSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set climber motor PID constants
    var slot0Configs = climberConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply climber motor configuration and initialize position to 0
    StatusCode climberStatus = climberMotor.getConfigurator().apply(climberConfigs, 0.050);
    if (!climberStatus.isOK()) {
      System.err.println("Could not apply climber motor configs. Error code: " + climberStatus.toString());
      DriverStation.reportError("Could not apply climber motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + climberStatus.toString());
    }
    climberMotor.getConfigurator().setPosition(0);

    // Create servos
    brakeServo = new Servo(brakeServoID);
    rampServo1 = new Servo(rampServo1ID);
    rampServo2 = new Servo(rampServo2ID);

  }

  @Override
  public void periodic() {
    currentPosition = climberMotor.getPosition().refresh().getValueAsDouble();

    // Put motor status on the Smart Dashboard
    SmartDashboard.putNumber("Climber Motor Amps", climberMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber Motor Volts", climberMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ClimberMotor Position", climberMotor.getPosition().getValueAsDouble());

    // Checl for current limit and stop motor
    if (climberMotor.getStatorCurrent().getValueAsDouble() > CURRENT_LIMIT) {
      stopClimber();
    }

  }

  /**
   * Extend the climber to prepare for climb
   */
  public void extendClimber() {
    climberMotor.setControl(new PositionVoltage(ClimberPositions.Extend).withSlot(0));
    holdPosition = false;
    brakeServo.setAngle(0);
    rampServo1.setAngle(180);
    rampServo2.setAngle(180);
  }

  /**
   * Retract the climber to climb the robot
   */
  public void retractClimber() {
    climberMotor.setControl(new PositionVoltage(ClimberPositions.Retract).withSlot(0));
    holdPosition = false;
    brakeServo.setAngle(0);
    rampServo1.setAngle(0);
    rampServo2.setAngle(0);
  }

  /**
   * Return climber to its home (starting) position
   */
  public void homeClimber() {
    climberMotor.setControl(new PositionVoltage(ClimberPositions.Home).withSlot(0));
    holdPosition = false;
    brakeServo.setAngle(0);
    rampServo1.setAngle(0);
    rampServo2.setAngle(0);
  }

  public void runClimber(double direction) {
    if (Math.abs(direction) < 0.01) {
      if (holdPosition) {
        SmartDashboard.putNumber("Climber H Pos", currentPosition);
        SmartDashboard.putBoolean("Climber Hold", true);
        climberMotor.setControl(new PositionVoltage(currentPosition).withSlot(0));
        holdPosition = false;
      }
    } else {
      brakeServo.setAngle(0);
      SmartDashboard.putBoolean("Climber Hold", false);
      holdPosition = true;
      climberMotor.setControl(new DutyCycleOut(direction));
    }
  }

  /**
   * Get the current draw for the climber motor
   * 
   * @return Motor amps
   */
  public double getMotorAmps() {
    return climberMotor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * 
   * Stop running the climber motor
   * 
   */
  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void killMotor() {
    stopClimber();
  }

  public void zeroEncoder() {
    climberMotor.setPosition(0);
  }

  /**
   * Command to climb with the climber
   * 
   * Needs to be called twice: the first time is to extend it, and the second is to retract
   */
  public class Climb extends Command {

    enum State {
      /**
       * The initial state, we haven't started climbing
       */
      Home,
      /**
       * The climber is extended and ready to climb
       */
      Extended,
      /**
       * We have climbed
       */
      Retracted
    }

    State state = State.Home;

    @Override
    public void initialize() {
      switch (state) {
        case Home:
          SmartDashboard.putString("Climber State", "Extended");
          extendClimber();
          state = State.Extended;
          break;
        case Extended:
          SmartDashboard.putString("Climber State", "Retracted");
          retractClimber();
          state = State.Retracted;
          break;
        case Retracted:
          SmartDashboard.putString("Climber State", "Home");
          homeClimber();
          state = State.Home;
          break;
      }
    }

    @Override
    public boolean isFinished() {
      double target = 0;
      switch (state) {
        case Home: target = ClimberPositions.Home; break;
        case Extended: target = ClimberPositions.Extend; break;
        case Retracted: target = ClimberPositions.Retract; break;
      }
      return Math.abs(target - currentPosition) < 5;
    }

    @Override
    public void end(boolean interrupted) {
      if (!interrupted && state.equals(State.Retracted))
        brakeServo.setAngle(30);
    }
  }
}
