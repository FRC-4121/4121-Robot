// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter extends SubsystemBase {
   //attributes; variables
   private WPI_TalonFX shooterMotor;
   private double speed; 

     //Limit switch to count balls on board
  private DigitalInput ShooterSwitch = new DigitalInput(2);
  
  private MedianFilter rpmFilter;

   /** Creates a new Shooter. */
  public Shooter(int shooterId) {
    shooterMotor = new WPI_TalonFX(shooterId);
    //Set up encoder for speed control
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxShoot, kTimeoutMsShoot);

    //Config speed PID
    shooterMotor.config_kP(kPIDLoopIdxShoot, kP_Shoot, kTimeoutMsShoot);
    shooterMotor.config_kI(kPIDLoopIdxShoot, kI_Shoot, kTimeoutMsShoot);
    shooterMotor.config_kD(kPIDLoopIdxShoot, kD_Shoot, kTimeoutMsShoot);
    shooterMotor.config_kF(kPIDLoopIdxShoot, kF_Shoot, kTimeoutMsShoot); 

    //Set filter for smoothing rpm
    rpmFilter = new MedianFilter(FILTER_WINDOW_SIZE);

  }

  //methods
  public void shooterStop(){

    shooterMotor.set(ControlMode.PercentOutput, 0);
    //processorMotor.set(ControlMode.PercentOutput, 0);

  }

  public void shooterRun(){
  
    shooterMotor.set(ControlMode.PercentOutput, -0.36);
    //~30 percent for low goal right against the goal
    //40 decent from one robot length away, 43 without having the shooter warm up to the rpm
    //with 6 feet away, 35 percent or just over 2000 rpm with two balls
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  }

  public void shooterRun(double speed){
  
    shooterMotor.set(ControlMode.PercentOutput, speed);
    //~30 percent for low goal
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  } 
  
  public void shooterRun(int percent){
  
    shooterMotor.set(ControlMode.PercentOutput, percent/100.0);
    //~30 percent for low goal
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  } 

  public void shootRPM(double rpm){
    //Speed for velocity control is encoder units per 100ms/ so divide by 600 instead of just 60
    speed = rpm * 2048 / 600; // 2048 is the amount of raw sensor units in a rotation
    shooterMotor.set(ControlMode.Velocity, speed);
  }

  public double getRPM()
  {
    return Math.abs(rpmFilter.calculate(shooterMotor.getSelectedSensorVelocity() * 600 / 2048));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
