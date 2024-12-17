// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.MechanismConstants.*;

/**
 * 
 * @deprecated I didn't quite migrate this correctly so it may break
 * 
 */

@Deprecated
public class AutoRunShooterSpeaker extends AutoCommand {
  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private boolean canShoot;

  public AutoRunShooterSpeaker(Shooter shoot, Processor process, Intake in, double endTime) {
    super(endTime);

    shooter = shoot;
    processor = process;
    intake = in;
    canShoot = false;

    addRequirements(shooter, processor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if ((timer.get() - startTime > 0.5) && Math.abs(CurrentShooterAngle -
    // IdleAngle) > ShooterAngleTolerance) {

    // canShoot = true;
    // }

    shooter.runShooterAuto(TopShootSpeakerSpeed, BottomShootSpeakerSpeed);

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
}