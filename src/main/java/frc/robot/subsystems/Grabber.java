// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import frc.robot.subsystems.Pneumatics;

public class Grabber extends SubsystemBase {
  
  //I'm not sure which motor controller will end up being used for the wrist, it is a different type of motor
  private WPI_TalonFX wrist = new WPI_TalonFX(WristID);

  private Pneumatics pneumatic = new Pneumatics();
  
  /** Creates a new Grabber. */
  public Grabber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Method to command the wrist motor to move by passing in a speed
  public void moveWrist(double speed) {

    wrist.set(speed);
  
  }

  // Clamp down on the game piece
  public void grab() {

    pneumatic.extend();// Will require testing to see if this needs to extend or retract

  }

  // Let go of the game piece
  public void letGo() {

    pneumatic.retract();// Will require testing to see if this needs to extend or retract

  }
  
}
