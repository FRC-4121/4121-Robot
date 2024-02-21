// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.Timer;

public class RunShooterAmp extends Command {

  // Declare local variables
  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private Timer timer;
  private double startTime;
  private double stopTime;
  private boolean canShoot;

  public RunShooterAmp(Shooter shoot, Processor process, Intake in, double endTime) {

    // Initialize local variables
    shooter = shoot;
    processor = process;
    intake = in;
    stopTime = endTime;

    // Set subsystem requirements
    addRequirements(shooter, processor, intake);

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

    if (timer.get() - startTime > shooterDelay) {

      canShoot = true;

    }

    shooter.runShooterAuto(TopShootAmpSpeed, BottomShootAmpSpeed);

    if (canShoot) {

      processor.runProcessor(0.8);
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
