// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterAngle extends SubsystemBase {
  
  //Motors
  private CANSparkMax pivotMotor;

  //Motor ID's
  private final int pivotMotorID = 15;//need to find

  //Limit switches to know when we have rotated all the way down or all the way up
  private DigitalInput TopSwitch = new DigitalInput(2);
  private DigitalInput BottomSwitch = new DigitalInput(3);

  //Encoder
  private final AbsoluteEncoder angleEncoder;
  
  //Rev PID controller
  private final SparkPIDController anglePID;

  //PID values
  private final double angleKP = 1;
  private final double angleKI = 0;
  private final double angleKD = 0;
  private final double angleFF = 0;
  private final double angleMinOutput = -1;
  private final double angleMaxOutput = 1;

  //Velocity factor in degrees per second
  private final double encoderVelocityFactor = 6.0; // (360/60)

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {

    pivotMotor = new CANSparkMax(pivotMotorID,MotorType.kBrushless);
    angleEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    anglePID = pivotMotor.getPIDController();
    
    
    anglePID.setFeedbackDevice(angleEncoder);
    angleEncoder.setVelocityConversionFactor(encoderVelocityFactor);
    anglePID.setP(angleKP);
    anglePID.setI(angleKI);
    anglePID.setD(angleKD);
    anglePID.setFF(angleFF);
    anglePID.setOutputRange(angleMinOutput,angleMaxOutput);

  }

  public void runPivot(double speed){
    pivotMotor.set(speed);
  }

  public Boolean getTopSwitch(){
    return TopSwitch.get();
  }

  public Boolean getBottomSwitch(){
    return BottomSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
