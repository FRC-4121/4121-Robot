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
  private Compressor compressor = new Compressor(ControlModuleID,PneumaticsModuleType.CTREPCM);

  //Module and channels need to be found and set in constants
  private DoubleSolenoid grabber = new DoubleSolenoid(ControlModuleID, PneumaticsModuleType.CTREPCM, GrabOpenChannelID, GrabCloseChannelID);

  //Solenoid for brake
  private DoubleSolenoid brake = new DoubleSolenoid(ControlModuleID, PneumaticsModuleType.CTREPCM, BrakeOpenChannelID, BrakeCloseChannelID);
  
  /** Creates a new Pneumatics. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Extend with the solenoid
  public void grab() {
    grabber.set(Value.kReverse);
  }

  //Retract with the solenoid
  public void letGo() {
    grabber.set(Value.kForward);
  }

  //Apply the brake
  public void releaseBrake() {
    brake.set(Value.kReverse); 
  }

  //Release the brake
  public void applyBrake() {
    brake.set(Value.kForward); 
  }
}
