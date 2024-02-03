// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterAngle extends SubsystemBase {
  
  //Motors
  private CANSparkMax pivotMotor;

  //Duty Cycle Encoder (for use w/ Rev Through Bore Encoder)
  public final DutyCycleEncoder encoder;

  //Limit switches to know when we have rotated all the way down or all the way up
  private DigitalInput TopSwitch = new DigitalInput(2);
  private DigitalInput BottomSwitch = new DigitalInput(3);

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {

    // Create a new Spark MAX controller for shooter angle
    pivotMotor = new CANSparkMax(kPivotMotorID,MotorType.kBrushless);

    // Factory reset so we can get the Spark MAX to a know state before
    // configuring.  Useful if we swap out a controller.
    pivotMotor.restoreFactoryDefaults();

    // Set brake mode and current limit
    pivotMotor.setIdleMode(kAngleMotorIdleMode);
    pivotMotor.setSmartCurrentLimit(kAngleMotorCurrentLimit);

    //If something goes wrong, save configuration
    pivotMotor.burnFlash();

    //Create new Duty Cycle Encoder
    encoder = new DutyCycleEncoder(kAngleEncoderID);
    encoder.setDistancePerRotation(kDistancePerRotation);

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

  public double getEncoderDistance() {
    return encoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
