// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CClaw extends SubsystemBase {

  private final double DRIVE_DEADBAND = 0.001;
  private final double CURRENT_LIMIT = 100;

  //Declare motor variables
  private TalonFX rotationMotor;
  private TalonFX intakeMotor;

  //Declare Phoenix PID controller gains
  private double rotate_kG = 0.0;
  private double rotate_kS = 0.1;
  private double rotate_kV = 0.1;
  private double rotate_kA = 0.0;
  private double rotate_kP = 0.1;
  private double rotate_kI = 0.0;
  private double rotate_kD = 0.0;

  //Declare motor output requests
  private final PositionVoltage requestPosition = new PositionVoltage(0.0);
  private final DutyCycleOut requestDuty = new DutyCycleOut(0.0);  

  //Declare motor IDs
  private final int rotationMotorID = 15; 
  private final int intakeMotorID =  16;

  public CClaw() {
    rotationMotor = new TalonFX(rotationMotorID);
    intakeMotor = new TalonFX(intakeMotorID);
   
    //Configure the Rotation Motor
    var rotateConfigs = new TalonFXConfiguration();

    //set rotate motor output configuration
    var rotateOutputConfigs = rotateConfigs.MotorOutput;
    rotateOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rotateOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    rotateOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    //Set rotate motor feedback sensor
    var rotateSensorConfig = rotateConfigs.Feedback;
    rotateSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    //Set rotate motor configure limits
    var rotateLimitConfig = rotateConfigs.CurrentLimits;
    rotateLimitConfig.StatorCurrentLimitEnable = true;
    rotateLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    //Set rotate motor PID constants
    var Slot0Configs = rotateConfigs.Slot0;
    Slot0Configs.kG = rotate_kG;
    Slot0Configs.kS = rotate_kS;
    Slot0Configs.kV = rotate_kV;
    Slot0Configs.kA = rotate_kA;
    Slot0Configs.kP = rotate_kP;
    Slot0Configs.kI = rotate_kI;
    Slot0Configs.kD = rotate_kD;

    //Apply rotate motor configuration and initialize position to 0
    StatusCode rotationStatus = rotationMotor.getConfigurator().apply(rotateConfigs, 0.050);
    if (!rotationStatus.isOK()) {
      System.out.println("Could not apply rotation motor configs. Error code: " + rotationStatus.toString());
      DriverStation.reportError("Could not apply rotation motor configs.", false);
    } else {
      System.out.println("Successfully applied rotation motor configs. Error code: " + rotationStatus.toString());
    }
    rotationMotor.getConfigurator().setPosition(0);

    //Configure the Intake Motor
    var intakeConfigs = new TalonFXConfiguration();

    
    //set intake motor output configuration
    var intakeOutputConfigs = intakeConfigs.MotorOutput;
    intakeOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    intakeOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    //Set intake motor feedback sensor
    var intakeSensorConfig = intakeConfigs.Feedback;
    intakeSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    //Set intake motor configure limits
    var intakeLimitConfig = intakeConfigs.CurrentLimits;
    intakeLimitConfig.StatorCurrentLimitEnable = true;
    intakeLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    //Apply intake motor configuration and initialize position to 0
    StatusCode intakeStatus = rotationMotor.getConfigurator().apply(intakeConfigs, 0.050);
    if (!intakeStatus.isOK()) {
      System.out.println("Could not apply intake motor configs. Error code: " + intakeStatus.toString());
      DriverStation.reportError("Could not apply intake motor configs.", false);
    } else {
      System.out.println("Successfully applied intake motor configs. Error code: " + intakeStatus.toString());
    }
    intakeMotor.getConfigurator().setPosition(0);

  }

  /**
   * Rotate claw to position
   * 
   * @param rotation  position in encoder units
   */
  public void setRotation(double rotation) {
    rotationMotor.setControl(requestPosition.withPosition(rotation));
  }


  @Override
  public void periodic(){
    // This method will be called once per scheduler run
  }
}
