// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
  
  // Declare class variables
  private CANSparkMax wrist;
  private RelativeEncoder wrist_encoder;
  private SparkMaxPIDController wrist_controller;

  /** Creates a new Wrist. */
  public Wrist() {

    // Set up motor controller
    wrist = new CANSparkMax(WristID,CANSparkMax.MotorType.kBrushless);
    wrist_encoder = wrist.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    wrist.restoreFactoryDefaults();

    // Set up PID controller for wrist position
    wrist_controller = wrist.getPIDController();
    wrist_controller.setFeedbackDevice(wrist_encoder);
    wrist_controller.setP(wrist_kP);
    wrist_controller.setI(wrist_kI);
    wrist_controller.setD(wrist_kD);
    wrist_controller.setIZone(wrist_kIz);
    wrist_controller.setFF(wrist_kFF);
    wrist_controller.setOutputRange(wrist_MinOutput, wrist_MaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Move the wrist at the desired speed
  public void move(double speed) {

    wrist.set(speed);

  }

  // Move the wrist to the encoder position
  public void moveToPosition(double rotations) {

    wrist_controller.setReference(rotations, CANSparkMax.ControlType.kPosition);

  }

  // Zero the current encoder position
  public void zeroEncoder() {

    wrist_encoder.setPosition(0.0);

  }

  // Read the current encoder position
  public double getEncoderPos() {

    return wrist_encoder.getPosition();

  }

  // Stop the wrist
  public void stopWrist() {

    move(0);
    
  }
}
