// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
   //attributes; variables
   private WPI_TalonSRX intakeMotor; 
  private CANSparkMax shooterMotorTop;
  private CANSparkMax shooterMotorBottom;

   /** Creates a new Shooter. */
  public Shooter() {
    intakeMotor = new WPI_TalonSRX(2);
    shooterMotorTop = new CANSparkMax(7, MotorType.kBrushless);
    shooterMotorBottom = new CANSparkMax(8, MotorType.kBrushless);

  }

  //methods


  public void runShooter(double speed){
  
    shooterMotorTop.set(-0.65*speed);
    shooterMotorBottom.set(0.65*speed);
  } 
 
  public void runShooterAmp(double speed){
    shooterMotorTop.set(-0.22*speed);
    shooterMotorBottom.set(0.04*speed);
  }

  public void runIntake(double speed){
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
