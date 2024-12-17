// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

public class Pneumatics extends SubsystemBase {

  // Module and channels need to be found and set in constants

  // Channel needs to be found, should be a constant
  private Compressor compressor = new Compressor(ControlModuleID, PneumaticsModuleType.REVPH);

  // Solenoid for climber
  private DoubleSolenoid climber = new DoubleSolenoid(ControlModuleID, PneumaticsModuleType.REVPH, ClimberOpenChannelID,
      ClimberCloseChannelID);

  // Pressure Sensor
  private AnalogInput pressureSensor = new AnalogInput(0);

  public static enum ClimberDirection {
    EXTEND,
    RETRACT,
  }

  /** Creates a new Pneumatics. */
  public Pneumatics() {
  }

  @Override
  public void periodic() {

    // Send compressor information to dashboard
    SmartDashboard.putBoolean("Compressor Active", compressor.isEnabled());
    SmartDashboard.putNumber("Compressor Current", compressor.getCurrent());

  }

  // Extend the climber
  public void extendClimber() {
    climber.set(Value.kReverse); // Need to find if this is forward or back
  }

  // Retract the climber
  public void retractClimber() {
    climber.set(Value.kForward);// Need to find if this is forward or back
  }

  public void runClimber(ClimberDirection dir) {
    switch (dir) {
      case EXTEND:
        climber.set(Value.kReverse);
        break;
      case RETRACT:
        climber.set(Value.kForward);
        break;
    }
  }

  public boolean isExtended() {
    return climber.get() == Value.kReverse;
  }

  // Get pressure from sensor
  public double getPressure() {
    return (250 * (pressureSensor.getVoltage() / pressureSensorVoltage)) - 25;// Equation for converting output voltage
                                                                              // to psi
  }

}
