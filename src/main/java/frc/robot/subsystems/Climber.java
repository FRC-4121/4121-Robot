// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {  
  private TalonFX climberMotor;
  private DigitalInput limitSwitch;
  private boolean climbing;

  private static final int climberId = -1;
  private static final int switchChannel = -1;
  

  public Climber() {
    climberMotor = new TalonFX(climberId);
    limitSwitch = new DigitalInput(switchChannel);
  }

  public void climb() {
    climbing = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberMotor.set(climbing && !limitSwitch.get() ? 1 : 0);
  }
}
