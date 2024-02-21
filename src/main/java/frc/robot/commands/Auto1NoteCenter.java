// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.wpilibj.Timer;

public class Auto1NoteCenter extends Command {

  // Declare local variables
  private SwerveDriveWPI swerve;
  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private NetworkTableQuerier ntable;

  private Timer timer;
  private double startTime;
  private double stopTime;

  private boolean canShoot;


  /** Creates a new Auto1NoteCenter. */
  public Auto1NoteCenter(SwerveDriveWPI drive, Shooter, shoot, Processor process, Intake in, NetworkTableQuerier table, double endtime) {

    // Set local variables
    swerve = drive;
    shooter = shoot;
    processor = process;
    intake = in;
    ntable = table;
    stopTime = endtime;

    // Set subsystem requirements
    addRequirements(serve, shooter, processor, intake);

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


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop all motors
    swerve.stopDrive();
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
