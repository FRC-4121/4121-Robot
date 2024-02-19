                                                                   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.MechanismConstants.*;


public class Intake extends SubsystemBase {

  //create intake motor
  private WPI_TalonFX intakeMotor;


  /** Creates a new intake. */
  public Intake() {

    intakeMotor = new WPI_TalonFX(IntakeMotorID);

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
