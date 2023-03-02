// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import static frc.robot.Constants.*;

public class LED extends SubsystemBase {
  
  Spark ledController;
  
  /** Creates a new LED. */
  public LED() {

    ledController = new Spark(0); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(double color) {
    
    ledController.set(color);
  }
}
