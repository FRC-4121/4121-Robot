// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can:) :( ) modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/** Add your docs here. */
public class PhotoElecSensor {
    DigitalInput photoSensor;

    public PhotoElecSensor() {
        photoSensor = new DigitalInput(5);
    }

    // Method for if we have a ring,if it's blocked, we have a ring
    public void isNoteOnBoard() {
        Constants.noteOnBoard = !photoSensor.get(); // a note is on board if the photosensor is blocked
    }
}
