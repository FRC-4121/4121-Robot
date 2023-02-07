// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  
  private WPI_TalonFX extend = new WPI_TalonFX(Extend);
  private WPI_TalonFX rotate1 = new WPI_TalonFX(Rotate);
  private WPI_TalonFX rotate2 = new WPI_TalonFX(Rotate);

  /** Creates a new Arm. */
  public Arm() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm(double speed){
    
    extend.set(speed); //Need to test to see if we have to invert this
  }

  public void retractArm(double speed){

    extend.set(-speed); //Need to test to see if we have to invert this
  }

  //rotate the arm back and forth, the two motors need to run in inverse directions
  public void rotateArm(double speed){

    rotate1.set(speed);
    rotate2.set(-speed);
  }
}
