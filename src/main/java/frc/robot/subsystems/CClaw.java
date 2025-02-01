// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CClaw extends SubsystemBase {
  private TalonFX CClawMotor;

  private static final Slot0Configs pidConfig = new Slot0Configs(); //TODO: assign parameters
  private static final PositionVoltage command = new PositionVoltage(0.0);

  private static final int CClawId = -1; //TODO: assign ids

  private static final double ROTATION_SCALE = 1;
  private static final double ROTATION_BASE = 0; 

  public CClaw() {
    CClawMotor = new TalonFX(CClawId);
   
    var config = CClawMotor.getConfigurator();
    config.apply(pidConfig);

  }

  public void setRotation(double rotation) {
    CClawMotor.setControl(command.withPosition(rotation * ROTATION_SCALE + ROTATION_BASE));
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
  }
}
