// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Intake;

import frc.robot.Constants.MechanismConstants;
import static frc.robot.Constants.MechanismConstants.shooterDelay;


public class RunShooterTrap extends TimeoutCommand {

  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private boolean canShoot;

  /** Creates a new RunShooterSpeaker. */
  public RunShooterTrap(Shooter shoot, Processor process, Intake in, double endTime) {
    super(endTime);
    shooter = shoot;
    processor = process;
    intake = in;

    addRequirements(shooter, processor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    canShoot = false;
    MechanismConstants.PauseAutoPosition = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (currentTime() > shooterDelay)
      canShoot = true;

    shooter.runShooterAuto(MechanismConstants.TopShooterTrapSpeed, MechanismConstants.BottomShooterTrapSpeed);

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

    MechanismConstants.PauseAutoPosition = false;
  }
}
