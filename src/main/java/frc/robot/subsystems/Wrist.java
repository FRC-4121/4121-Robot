// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase {
  
  private WPI_TalonSRX wrist = new WPI_TalonSRX(WristID);

  /** Creates a new Wrist. */
  public Wrist() {

    wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double speed){

    wrist.set(speed);
  }

  public double getPosition(){

    return wrist.getSelectedSensorPosition();
  }
}
