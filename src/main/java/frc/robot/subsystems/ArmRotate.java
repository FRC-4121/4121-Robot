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
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class ArmRotate extends SubsystemBase {
  
  private WPI_TalonFX rotateMaster;
  private WPI_TalonFX rotateSlave;


  
  /** Creates a new ArmRotate. */
  public ArmRotate() {

    //Defining Motors
    rotateMaster = new WPI_TalonFX(Rotate1);
    rotateSlave = new WPI_TalonFX(Rotate2);

    //Make sure that the configs are default before we set things
    rotateMaster.configFactoryDefault();
    rotateSlave.configFactoryDefault();

    //Make sure motors are in brake mode
    rotateMaster.setNeutralMode(NeutralMode.Brake);
    rotateSlave.setNeutralMode(NeutralMode.Brake);

    //Set the slave to follow the master
    rotateSlave.follow(rotateMaster);
    
    //Setting motor directions, making sure they are in opposite directionz
    rotateSlave.setInverted(!kMotorInvert);
    rotateMaster.setInverted(kMotorInvert);

    //Config encoders
    rotateMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    rotateSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);

    //Configure deadband for PID control 
    rotateMaster.configNeutralDeadband(0.001);
    rotateSlave.configNeutralDeadband(0.001);

    //Config PID control variables
    rotateMaster.selectProfileSlot(0,0);
    rotateMaster.config_kF(0,rotateGains.kF, kTimeoutMsDrive);
    rotateMaster.config_kP(0,rotateGains.kP, kTimeoutMsDrive);
    rotateMaster.config_kI(0,rotateGains.kI, kTimeoutMsDrive);
    rotateMaster.config_kD(0,rotateGains.kD, kTimeoutMsDrive);

    rotateMaster.configMotionCruiseVelocity(rotateVelocity,kTimeoutMsDrive);
    rotateMaster.configMotionAcceleration(rotateAcceleration,kTimeoutMsDrive);

  }


  public void rotate(double speed) {
    
    rotateMaster.set(speed); //Need to test to see if we have to invert this
  }

  //Position is in encoder units
  public void rotateToPosition(double position) {

    //rotateMaster.set(TalonFXControlMode.MotionMagic, position);    
    rotateMaster.set(TalonFXControlMode.Position, position);

  }

  public double getMasterEncoder() {

    return rotateMaster.getSelectedSensorPosition();
  }

  public double getSlaveEncoder() {

    return rotateSlave.getSelectedSensorPosition();
  }

  public void zeroEncoder() {

    rotateMaster.setSelectedSensorPosition(0);
    rotateSlave.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
