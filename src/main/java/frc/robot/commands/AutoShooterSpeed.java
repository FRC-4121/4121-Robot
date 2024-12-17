// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.MechanismConstants;
import frc.robot.ExtraClasses.NetworkTableQuerier;

public class AutoShooterSpeed extends Command {

  // Declare local variables
  private ShooterMode mode;
  private Shooter shooterMotor;
  private NetworkTableQuerier ntable;

  private double tagsFound;
  private int closestTag;
  private double closestDistance;
  private int tagID;
  private double tagDistance;
  private boolean isMyTag;

  /**
   * Mode for the shooter
   */
  public static enum ShooterMode {
    IDLE,
    SPEAKER,
    AMP,
  }

  /** Creates a new AutoShooterSpeed. */
  public AutoShooterSpeed(Shooter shoot) {
    // Set local variables
    shooterMotor = shoot;

    addRequirements(shooterMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check current mode to determine correct shooter speeds
    switch (mode) {
      case SPEAKER: 
        shooterMotor.runShooterAuto(MechanismConstants.TopShootSpeakerSpeed, MechanismConstants.BottomShootSpeakerSpeed);
        break;
      case AMP: 
        shooterMotor.runShooterAuto(MechanismConstants.TopShootAmpSpeed, MechanismConstants.BottomShootAmpSpeed);
        break;
      case IDLE: 
        shooterMotor.runShooterAuto(MechanismConstants.TopShootIdleSpeed, MechanismConstants.BottomShootIdleSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMotor.runShooterAuto(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command runs continuously
    return false;
  }

  public ShooterMode getMode() {
    return mode;
  }

  public void setMode(ShooterMode mode) {
    this.mode = mode;
  } 
}
