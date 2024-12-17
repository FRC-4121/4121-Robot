// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants;

public class AutoShooterAmpPos extends AutoCommand {
  private ShooterAngle shooterAngle;
  private Shooter shooter;
  private Processor processor;
  private Intake intake;

  /** Creates a new AutoShooterAmpPos. */
  public AutoShooterAmpPos(ShooterAngle shooterAngle, Shooter shooter, Processor processor, Intake intake,
      double endTime) {
    super(endTime);
    this.shooter = shooter;
    this.shooterAngle = shooterAngle;
    this.processor = processor;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!Constants.noteOnBoard) {
      // Run the shooter at the correct Amp Speeds
      shooter.runShooterAuto(MechanismConstants.TopShootAmpSpeed, MechanismConstants.BottomShootAmpSpeed);

      // Check to see if we are at the amp angle
      if ((Math.abs(shooterAngle.getCurrentAngle() - MechanismConstants.AmpAngle) < MechanismConstants.ShooterAngleTolerance)
          && shooterAngle.getTopSwitch() == false) {
        shooterAngle.runPivot(MechanismConstants.AngleMotorSpeed);
      } else {
        shooterAngle.runPivot(0);
        processor.runProcessor(0.5);
        intake.runIntake(0.5);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runShooterSpeaker(0);
    processor.runProcessor(0.0);
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.noteOnBoard || Constants.killAuto || super.isFinished();
  }
}
