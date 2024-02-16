// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can:) :( ) modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class PhotoElecSensor {

    DigitalInput photoSensor;
    
    public PhotoElecSensor() {

        photoSensor = new DigitalInput(0);

    }

    //Method for if we have a ring,if it's blocked, we have a ring
    public void isNoteOnBoard() {

        if(photoSensor.get())
        {
            noteOnBoard = false;//If the photosensor is not blocked, a ring is not on board
        } else{
            noteOnBoard = true;//If the photosensor is blocked, a ring is on board
        }
        
    }
}
