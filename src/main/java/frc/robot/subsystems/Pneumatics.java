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


public class Pneumatics extends SubsystemBase {
  
  //Channel needs to be found, should be a constant
  private Compressor compressor = new Compressor(0,PneumaticsModuleType.CTREPCM);

  //Module and channels need to be found and set in constants
  private DoubleSolenoid shifter = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, ForwardChannelID, ReverseChannelID);
  
  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Extend with the solenoid
  public void extend(){
    shifter.set(Value.kReverse);
  }

  //Retract with the solenoid
  public void retract(){
    shifter.set(Value.kForward);
  }
}
