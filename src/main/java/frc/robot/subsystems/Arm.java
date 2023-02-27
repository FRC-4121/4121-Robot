// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;

public class Arm extends SubsystemBase {
  
  // Declare class level variables
  private WPI_TalonFX extend;
  private DigitalInput homeSwitch;
  private DigitalInput extendSwitch;

  /** Creates a new Arm. */
  public Arm() {

    // Create a new Talon FX motor controller
    extend = new WPI_TalonFX(Extend);

    // Configure motor controller brake mode
    extend.setNeutralMode(NeutralMode.Brake);

    // Configure motor controller encoders
    extend.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Create new limit switches
    homeSwitch = new DigitalInput(HomeSwitchID);
    extendSwitch = new DigitalInput(ExtendSwitchID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm(double speed){
    
    extend.set(speed); //Need to test to see if we have to invert this
  }

  // Stop the arm
  public void stopArm(){

    extend.set(0);

  }

  // Get current encoder position
  public double getExtendEncoder(){

    return extend.getSelectedSensorPosition();

  }

  // Set the current encoder position as the zero position
  public void zeroExtendEncoder(){

    extend.setSelectedSensorPosition(0);

  }

  // Get the current home limit switch value
  public boolean getHomeSwitchValue() {

    return homeSwitch.get();

  }

  // Get the current extend limit switch value
  public boolean getExtendSwitchValue() {

    return extendSwitch.get();

  }
}
