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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

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
    pivotMotor.setNeutralMode(NeutralMode.Brake);

    // Configure motor encoder
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,20);

    // Configure PID constants for motor control
    pivotMotor.config_kP(0, kShooterAngleKP, kTimeoutMsAngle);
    pivotMotor.config_kI(0, kShooterAngleKI, kTimeoutMsAngle);
    pivotMotor.config_kD(0, kShooterAngleKD, kTimeoutMsAngle);

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

  /**
   * 
   * Run the angle motor at a given speed
   * 
   * @param speed  Angle motor speed
   * 
   */
  public void runPivot(double speed){
    
    if(CurrentShooterAngle >= 50)
    {
      pivotMotor.set(speed*0.5);
    } else{
    pivotMotor.set(speed);
    }

    if(TopSwitch.get() == true)
    {
      if(speed > 0){
        pivotMotor.set(0);
      }
      pivotMotor.setSelectedSensorPosition(0);
    } else if(BottomSwitch.get() == true)
    {
      if (speed < 0) {
        pivotMotor.set(0);
      }
      MaxEncoderPos = getIntegratedValue();
    }

    CurrentShooterAngle = getCurrentAngle();

  }

  /**
   * 
   * Run the shooter to a specified angle
   * The motor controller PID controls the position
   * 
   * @param angle  Target angle
   * 
   */
  public void runPivotToAngle(double angle) {

    double targetPosition = -getEncoderForAngle(angle);
    pivotMotor.set(ControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, AngleMotorMinSpeed);

    CurrentShooterAngle = getCurrentAngle();
    CurrentShooterEncoder = getIntegratedValue();

  }

  /**
   * 
   * Read the value of the top limit switch
   * Switch = True will set encoder to zero and angle to max
   * 
   */
  public Boolean getTopSwitch(){

    return TopSwitch.get();

  }

  /**
   * 
   * Read the value of the bottom limit switch
   * Switch = True will set encoder to max and angle to min
   * 
   */
  public Boolean getBottomSwitch(){

    return BottomSwitch.get();

  }

  /**
   * 
   * Read the absolute encoder raw value
   * 
   */
  public double getEncoderValue() {

    return Math.abs(encoder.get());

  }

  /**
   * 
   * Read the absolute encoder position
   * 
   */
  public double getAbsoluteEncoderPosition(){

    return encoder.getAbsolutePosition();

  }

  /**
   * 
   * Read the integrated encoder absolute distance and average it
   */
  public double getEncoderDistance() {

    return angle_filter.calculate(Math.abs(encoder.getDistance()));

  }

  /**
   * 
   * Read the integrated encoder position and average it
   * 
   */
  public double getIntegratedValue() {

    return (int)Math.round(angle_filter.calculate(Math.abs(pivotMotor.getSelectedSensorPosition())));

  }

  /**
   * 
   * Zero the integrated encoder
   * 
   */
  public void zeroEncoder() {

    pivotMotor.setSelectedSensorPosition(0);

  }

  /**
   * 
   * Calculate the current shooter angle from the motor encoder position
   * 
   */
  public double getCurrentAngle() {

    return getIntegratedValue() * ((MinSpeakerAngle-MaxSpeakerAngle)/MaxEncoderPos) + MaxSpeakerAngle;

  }

  /**
   * 
   * Calculates the encoder value for a given angle
   * 
   * @param angle  Angle to calculate encoder position for
   * 
   */
  public double getEncoderForAngle(double angle) {

    return Math.round(MaxEncoderPos * (angle - MaxSpeakerAngle) / (MinSpeakerAngle - MaxSpeakerAngle));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
