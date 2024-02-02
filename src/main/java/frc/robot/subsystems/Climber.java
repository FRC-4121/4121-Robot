// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  
  private CANSparkMax climberMotor;

  //Motor ID's
  private final int climberID = 25;//need to find
  
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new CANSparkMax(climberID, MotorType.kBrushless);
  }

  public void RunClimber(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
