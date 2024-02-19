// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.MechanismConstants.*;


public class Shooter extends SubsystemBase {

   //attributes; variables
  private WPI_TalonFX shooterMotorTop;
  private WPI_TalonFX shooterMotorBottom;

   /** Creates a new Shooter. */
  public Shooter() {

    shooterMotorTop = new WPI_TalonFX(TopShooterID);
    shooterMotorBottom = new WPI_TalonFX(BottomShooterID);

  }

  // Run the shooter motors from an auto command
  public void runShooterAuto(double topSpeed, double bottomSpeed) {

    shooterMotorTop.set(topSpeed);
    shooterMotorBottom.set(bottomSpeed);

  }

  // Run the shooter motors at speaker scoring speeds
  // NOTE: 65% power is the max speed
  public void runShooter(double speed){
  
    shooterMotorTop.set(-0.65*speed);
    shooterMotorBottom.set(0.65*speed);

  } 
 
  // Run the shooter motors at Amp scoring speeds
  public void runShooterAmp(double speed){

    shooterMotorTop.set(-0.22*speed);
    shooterMotorBottom.set(0.04*speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
