// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

public class AutoShooterAngle extends Command {

  private ShooterAngle shootAngle;
  
  private double targetAngle;
  private Timer timer;
  private double startTime;
  private double stopTime;

  private PIDController wpiPIDController;
 
  /** Creates a new AutoShooterPos. */
  public AutoShooterAngle(ShooterAngle shoot, double angle, double endTime) {

    // Set local variables
    shootAngle = shoot;
    targetAngle = angle;
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

    

    // Determine new motor speed from PID controller
    double pidOutput = wpiPIDController.calculate(CurrentShooterAngle, targetAngle);
    if(pidOutput >1)
    {
      pidOutput = 1;
    } else if (pidOutput < -1)
    {
      pidOutput = -1;
    }

    // Run angle motor at new speed (as long as we aren't at bounds)
    if (Math.abs(CurrentShooterAngle - ShooterTargetAngle) > ShooterAngleTolerance) {

      double angleSpeed = AngleMotorSpeed * pidOutput;

      if (pidOutput > 0 && shootAngle.getTopSwitch() == false) {

        shootAngle.runPivot(AngleMotorMinSpeed + angleSpeed);

      }
      else if (pidOutput < 0 && shootAngle.getBottomSwitch() == false) {

        shootAngle.runPivot(-AngleMotorMinSpeed + angleSpeed);

      }

    } else {

      shootAngle.runPivot(0.0);

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
    } else if (Math.abs(CurrentShooterAngle - ShooterTargetAngle) < ShooterAngleTolerance){
      thereYet = true;
    } else if (stopTime <= time - startTime){
      thereYet = true;
    }

    return thereYet;

  }

}
