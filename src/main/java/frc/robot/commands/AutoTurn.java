// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.ExtraClasses.PIDControl;
import frc.robot.Constants;

public class AutoTurn extends AutoCommand {

  // Declare class variables
  double targetAngle;

  PIDControl pidControl;

  private final Drivetrain drivetrain;

  /** Creates a new AutoTurn. */
  public AutoTurn(Drivetrain drivetrain, double targetAngle) {
    super(60.0); // until we figure out what this was supposed to be
    this.drivetrain = drivetrain;
    this.targetAngle = targetAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    // Set up PID control
    pidControl = new PIDControl(Constants.kP_Turn, Constants.kI_Turn, Constants.kD_Turn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleCorrection = pidControl.run(drivetrain.getGyroAngle(), targetAngle);
    double motorOutput = angleCorrection * Constants.kAutoTurnSpeed;
    drivetrain.autoDrive(motorOutput, -motorOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (super.isFinished()) {
      return true;
    }
    double angleError = drivetrain.getGyroAngle() - targetAngle;
    return Math.abs(angleError) <= Constants.kTurnAngleTolerance;
  }
}
