// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAlignToTape extends CommandBase {
  
  // Declare class variables
  private final SwerveDrive drivetrain;
  private double startTime;
  private double stopTime;
  private NetworkTableQuerier ntables;
  private Timer timer;

  /** Creates a new AutoAlignToTape. */
  public AutoAlignToTape(SwerveDrive drive, double time, NetworkTableQuerier table) {
  
    drivetrain = drive;
    stopTime = time;
    ntables = table;

    addRequirements(drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Initialize variables
    timer = new Timer();
    timer.start();
    startTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
