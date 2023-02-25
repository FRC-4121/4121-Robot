// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class ArmRotate extends SubsystemBase {
  
  private WPI_TalonFX rotateMaster = new WPI_TalonFX(Rotate1);
  private WPI_TalonFX rotateSlave = new WPI_TalonFX(Rotate2);


  
  /** Creates a new ArmRotate. */
  public ArmRotate() {

    //Set the slave to follow the master
    rotateSlave.follow(rotateMaster);
    
    //Setting motor directions, making sure they are in opposite directionz
    rotateSlave.setInverted(!kMotorInvert);
    rotateMaster.setInverted(kMotorInvert);

    //Config encoders
    rotateMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    rotateSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);

  }


  public void rotate(double speed){
    
    rotateMaster.set(speed); //Need to test to see if we have to invert this
  }

  public double getMasterEncoder(){

    return rotateMaster.getSelectedSensorPosition();
  }

  public double getSlaveEncoder(){

    return rotateSlave.getSelectedSensorPosition();
  }

  public void zeroEncoder(){

    rotateMaster.setSelectedSensorPosition(0);
    rotateSlave.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
