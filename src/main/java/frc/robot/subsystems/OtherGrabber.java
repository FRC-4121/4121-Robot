// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.Pneumatics;

public class OtherGrabber extends SubsystemBase {
  
  //Motor is a 550, will probably use this
  private CANSparkMax intake = new CANSparkMax(Intake,CANSparkMax.MotorType.kBrushless);

  private Pneumatics pneumatic = new Pneumatics();

  /** Creates a new OtherGrabber. */
  public OtherGrabber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed){

    intake.set(speed);
  }

  public void stopIntake(){

    intake.set(0);
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
