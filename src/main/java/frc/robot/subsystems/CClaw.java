// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CClaw extends SubsystemBase {

  private final double DRIVE_DEADBAND = 0.001;
  private final double CURRENT_LIMIT = 100;

  //Declare motor variables
  private TalonFX rotationMotor;
  private TalonFX intakeMotor;

  //Declare motor output requests
  private final PositionVoltage command = new PositionVoltage(0.0);

  //Declare motor IDs
  private final int rotationMotorID = 15; 
  private final int intakeMotorID =  16;

  public CClaw() {
    rotationMotor = new TalonFX(rotationMotorID);
    intakeMotor = new TalonFX(intakeMotorID);
   
    //Configure the Rotation Motor
    var rotateConfigs = new TalonFXConfiguration();

    var rotateOutputConfigs = rotateConfigs.MotorOutput;
    rotateOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    rotateOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    rotateOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    var rotateSensorConfig = rotateConfigs.Feedback;
    rotateSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    var rotateLimitConfig = rotateConfigs.CurrentLimits;
    rotateLimitConfig.StatorCurrentLimitEnable = true;
    rotateLimitConfig.StatorCurrentLimit = CURRENT_LIMIT;

    



  }

  public void setRotation(double rotation) {
    rotationMotor.setControl(command.withPosition(rotation));
    intakeMotor.setControl(command.withPosition(rotation));
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
  }
}
