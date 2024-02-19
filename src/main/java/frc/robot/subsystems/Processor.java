// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import static frc.robot.Constants.MechanismConstants.*;

public class Processor extends SubsystemBase {

  // Declare local variables
  private WPI_VictorSPX processorMotor; 

  /** Creates a new Processor. */
  public Processor() {

    processorMotor = new WPI_VictorSPX(ProcessorMotorID);

  }

  // Run the processor motor
  public void runProcessor(double speed){

    processorMotor.set(speed);

  }

  // Stop the processor motor
  public void stopProcessor() {

    processorMotor.set(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
