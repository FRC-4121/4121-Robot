// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.BOTTOM_SHOOTER_ID;
import static frc.robot.Constants.TOP_SHOOTER_ID;

import com.ctre.phoenix.motorcontrol.InvertType;
import frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

  private WPI_TalonSRX shooterMotorTop;
  private WPI_TalonSRX shooterMotorBottom;

  public Shooter() {

    shooterMotorTop = new WPI_TalonSRX(TOP_SHOOTER_ID);
    shooterMotorBottom = new WPI_TalonSRX(BOTTOM_SHOOTER_ID);

    shooterMotorTop.setInverted(InvertType.None);
    shooterMotorBottom.setInverted(InvertType.None);

  }

  @Override
  public void periodic() {
    
  }

  public void runShooter(double topSpeed, double bottomSpeed) {

    shooterMotorTop.set(topSpeed);
    shooterMotorBottom.set(bottomSpeed);

  }

  public void stopShooter() {

    shooterMotorTop.set(0.0);
    shooterMotorBottom.set(0.0);

  }

}
