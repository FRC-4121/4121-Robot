                                                                   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake extends SubsystemBase {

  //create intake variables
  private int INTAKE_ID = 1;

  //create intake motor
  private CANSparkMax intakeMotor;


  /** Creates a new intake. */
  public Intake() {

    intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {

    intakeMotor.set(speed);

  }

  public void stopIntake() {

    intakeMotor.set(0);

  }
}
