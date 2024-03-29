// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AutoShooterBottom extends Command {

  private ShooterAngle shootAngle;
  
  private Timer timer;
  private double startTime;
  private double stopTime;

  private PIDController wpiPIDController;
 
  /** Creates a new AutoShooterPos. */
  public AutoShooterBottom(ShooterAngle shoot, double endTime) {

    // Set local variables
    shootAngle = shoot;
    stopTime = endTime;
    addRequirements(shootAngle);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Create PID controller
    wpiPIDController = new PIDController(kShooterAngleKP, kShooterAngleKI, kShooterAngleKD);
    wpiPIDController.setTolerance(1.0,5);

    // Initialize a new timer and get current time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angleSpeed = -0.2;

    if (shootAngle.getBottomSwitch() == false) {

          shootAngle.runPivot(-AngleMotorMinSpeed + angleSpeed);
          SmartDashboard.putNumber("Angle Input", -AngleMotorMinSpeed + angleSpeed);

        } else {

          shootAngle.runPivot(0);

        }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop the shooter angle motor
    shootAngle.runPivot(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean thereYet = false;
    // Get the current time
    double time = timer.get();  

    if (killAuto) {
      thereYet = true;
    } else if (shootAngle.getBottomSwitch() == true){
      thereYet = true;
    } else if (stopTime <= time - startTime){
      thereYet = true;
    }

    return thereYet;

  }

}
