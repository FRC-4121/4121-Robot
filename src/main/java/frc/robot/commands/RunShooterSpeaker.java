// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class RunShooterSpeaker extends Command {
  
  private Shooter shooter;
  private Timer timer = new Timer();
  private double startTime;
  private double delay;
  private boolean canShoot;
  
  /** Creates a new RunShooterSpeaker. */
  public RunShooterSpeaker(Shooter shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
      shooter = shoot;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();
    delay = 0.5;
    canShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.get() - startTime > delay) {
      canShoot = true;
    }

    shooter.runShooter(1);

    if (canShoot) {
      shooter.runIntake(1.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    shooter.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
