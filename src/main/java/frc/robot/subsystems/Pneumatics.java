// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Pneumatics extends SubsystemBase {
  
  //Module and channels need to be found and set in constants

  //Channel needs to be found, should be a constant
  private Compressor compressor = new Compressor(ControlModuleID,PneumaticsModuleType.CTREPCM);

  //Solenoid for climber
  private DoubleSolenoid climber = new DoubleSolenoid(ControlModuleID, PneumaticsModuleType.CTREPCM, ClimberOpenChannelID, ClimberCloseChannelID);
  
  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    
    // Send compressor information to dashboard
    SmartDashboard.putBoolean("Compressor Active", compressor.isEnabled());
    SmartDashboard.putNumber("Compressor Current", compressor.getCurrent());

  }

  //Extend the climber
  public void extendClimber(){
    climber.set(Value.kForward);//Need to find if this is forward or back
  }

  //Retract the climber
  public void retractClimber(){
    climber.set(Value.kReverse);//Need to find if this is forward or back
  }
}
