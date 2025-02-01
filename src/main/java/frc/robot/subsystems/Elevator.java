// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private TalonFX elevatorMasterMotor;
  private TalonFX elevatorSlaveMotor;

  private static final Slot0Configs pidConfig = new Slot0Configs(); // TODO: determine parameters
  private static final PositionVoltage command = new PositionVoltage(0.0);

  private static final int masterId = -1; // TODO: assign ids
  private static final int slaveId = -1;

  private static final double LEVEL_SCALE = 1;
  private static final double LEVEL_BASE = 0;

  public static final double L1_LEVEL = 0.0; // TODO: assign constants
  public static final double L2_LEVEL = 0.0;
  public static final double L3_LEVEL = 0.0;
  public static final double L4_LEVEL = 0.0;

  public Elevator() {
    elevatorMasterMotor = new TalonFX(masterId);
    elevatorSlaveMotor = new TalonFX(slaveId);

    elevatorSlaveMotor.setControl(new Follower(masterId, true));
    var config = elevatorMasterMotor.getConfigurator();
    config.apply(pidConfig);
  }

  public void setLevel(double level) {
    elevatorMasterMotor.setControl(command.withPosition(level * LEVEL_SCALE + LEVEL_BASE));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
