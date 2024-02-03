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
import static frc.robot.Constants.MechanismConstants.*;

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


  /** Creates a new ShooterAngle. */
  public ShooterAngle() {

    // Create a new Spark MAX controller for shooter angle
    pivotMotor = new CANSparkMax(pivotMotorID,MotorType.kBrushless);

    // Factory reset so we can get the Spark MAX to a know state before
    // configuring.  Useful if we swap out a controller.
    pivotMotor.restoreFactoryDefaults();

    // Set up encoder and PID controller
    angleEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    angleEncoder.setInverted(kAngleInverted);
    anglePID = pivotMotor.getPIDController();
    anglePID.setFeedbackDevice(angleEncoder);
    angleEncoder.setVelocityConversionFactor(kAngleEncoderVelocityFactor);
    angleEncoder.setPositionConversionFactor(kAngleEncoderPositionFactor);
    anglePID.setP(kShooterAngleKP);
    anglePID.setI(kShooterAngleKI);
    anglePID.setD(kShooterAngleKD);
    anglePID.setFF(kShooterAngleFF);
    anglePID.setOutputRange(kShooterAngleMinOutput,kShooterAngleMaxOutput);

    // Set brake mode and current limit
    pivotMotor.setIdleMode(kAngleMotorIdleMode);
    pivotMotor.setSmartCurrentLimit(kAngleMotorCurrentLimit);

    //If something goes wrong, save configuration
    pivotMotor.burnFlash();

  }

  public void runPivot(double speed){
    pivotMotor.set(speed);
  }

  //Takes in an angle in degrees and lets the spark max runt he motor up the the angle usign the PID
  public void autoRunPivot(double angle)
  {
    //convert angle to radians
    angle = (angle/360)*(Math.PI * 2);

    anglePID.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public double getPosition(){
    return angleEncoder.getPosition();
  }

  public void zeroPosition(){
    angleEncoder.setZeroOffset(kAngleEncoderOffest);
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
