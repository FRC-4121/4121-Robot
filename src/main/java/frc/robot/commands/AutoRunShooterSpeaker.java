// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.MechanismConstants.*;

import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * @deprecated I didn't quite migrate this correctly so it may break
 * 
 */

@Deprecated
public class AutoRunShooterSpeaker extends Command {
  
  
  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private boolean canShoot;
  
  /** Creates a new RunShooterSpeaker. */
  public AutoRunShooterSpeaker(Shooter shoot, Processor process, Intake in, double endTime) {

      shooter = shoot;
      processor = process;
      stopTime = endTime;
      intake = in;

      addRequirements(shooter, processor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Initialize a new timer and get current time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

    // Initialize local variables
    canShoot = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if ((timer.get() - startTime > 0.5) && Math.abs(CurrentShooterAngle - IdleAngle) > ShooterAngleTolerance) {

    //   canShoot = true;
    // } 

    shooter.runShooterAuto(TopShootSpeakerSpeed,BottomShootSpeakerSpeed);

    if (canShoot) {
      processor.runProcessor(0.5);
      intake.runIntake(0.5);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop all motors
    shooter.runShooterAuto(0.0, 0.0);
    processor.runProcessor(0.0);
    intake.runIntake(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // Initialize return flag
    boolean thereYet = false;

    // Get the current time
    double time = timer.get();

    if (stopTime <= time - startTime) {
      thereYet = true;
    }

    // Return flag
    return thereYet;

  }
}