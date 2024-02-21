// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.MedianFilter;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
public class ShooterAngle extends SubsystemBase {
  
  //Motors
  //private CANSparkMax pivotMotor;
  private WPI_TalonFX pivotMotor;

  //Duty Cycle Encoder (for use w/ Rev Through Bore Encoder)
  public final DutyCycleEncoder encoder;

  //Limit switches to know when we have rotated all the way down or all the way up
  private DigitalInput TopSwitch = new DigitalInput(0);
  private DigitalInput BottomSwitch = new DigitalInput(9);

  //Median Filter
  private MedianFilter angle_filter; 

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {

    // Create a new Spark MAX controller for shooter angle
    //pivotMotor = new CANSparkMax(kPivotMotorID,MotorType.kBrushless);
    pivotMotor = new WPI_TalonFX(kPivotMotorID);

    //COnfigure Motors encoder
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);

    // Factory reset so we can get the Spark MAX to a know state before
    // configuring.  Useful if we swap out a controller.
    //pivotMotor.restoreFactoryDefaults();

    // Set brake mode and current limit
    //pivotMotor.setIdleMode(kAngleMotorIdleMode);
    //pivotMotor.setSmartCurrentLimit(kAngleMotorCurrentLimit);

    //If something goes wrong, save configuration
    //pivotMotor.burnFlash();

    //Create new Duty Cycle Encoder
    encoder = new DutyCycleEncoder(kAngleEncoderID);
    encoder.setDistancePerRotation(kDistancePerRotation);
    encoder.setDistancePerRotation(100.0);

    angle_filter = new MedianFilter(FILTER_WINDOW_SIZE);

  }

  public void runPivot(double speed){
    
    pivotMotor.set(speed);

    if(TopSwitch.get() == true)
    {
      pivotMotor.setSelectedSensorPosition(0);
    } else if(BottomSwitch.get() == true)
    {
      MaxEncoderPos = getIntegratedValue();
    }

    CurrentShooterAngle = getIntegratedValue() * ((MinSpeakerAngle-MaxSpeakerAngle)/MaxEncoderPos) + MaxSpeakerAngle;
  }

  //Should Zero encoder, set angle to max
  public Boolean getTopSwitch(){
    return TopSwitch.get();
  }

  //Should set encoder to max, angle to min
  public Boolean getBottomSwitch(){
    return BottomSwitch.get();
  }

  public double getEncoderValue() {
    return Math.abs(encoder.get());
  }

  public double getAbsoluteEncoderPosition(){
    return encoder.getAbsolutePosition();
  }

  public double getEncoderDistance() {
    return angle_filter.calculate(Math.abs(encoder.getDistance()));
  }

  public double getIntegratedValue() {
    return Math.abs(pivotMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
