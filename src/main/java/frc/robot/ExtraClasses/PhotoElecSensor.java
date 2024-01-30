// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import edu.wpi.first.wpilibj.DigitalInput;


/** Add your docs here. */
public class PhotoElecSensor {

    DigitalInput photoSensor;
    
    public PhotoElecSensor() {

        photoSensor = new DigitalInput(0);

    }

    public boolean isNotBlocked() {

        return photoSensor.get();
        
    }
}
