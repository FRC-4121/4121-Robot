// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm extends SubsystemBase {
  
  private WPI_TalonFX extend = new WPI_TalonFX(Extend);

  /** Creates a new Arm. */
  public Arm() {

    extend.setNeutralMode(NeutralMode.Brake);

    //configure encoders
    extend.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extendArm(double speed){
    
    extend.set(speed); //Need to test to see if we have to invert this
  }

  public void stopArm(){

    extend.set(0);
  }


  public double getExtendEncoder(){

    return extend.getSelectedSensorPosition();
  }

  public void zeroExtendEncoder(){

    extend.setSelectedSensorPosition(0);
  }
}
