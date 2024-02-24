// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED.*;

public class LED extends SubsystemBase {
  
  //Blinkin
  //Spark ledController;

  //Rio LED
  AddressableLED led;
  AddressableLEDBuffer buffer;
  
  /** Creates a new LED. */
  public LED() {

    //ledController = new Spark(7); 
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(120);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(int red, int green, int blue) {
    
    //ledController.set(color);
    for(int i = 0; i < buffer.getLength(); i++){
    buffer.setRGB(i,red,green,blue);
    }

    led.setData(buffer);
  }
}
