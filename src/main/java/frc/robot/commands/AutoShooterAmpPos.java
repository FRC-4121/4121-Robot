// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.*;

public class AutoShooterAmpPos extends Command {
  
  private ShooterAngle shooterAngle;
  private Shooter shooter;
  private Processor processor;
  private Intake intake;
  private Timer timer = new Timer();
  private double startTime;
  private double endTime;
  
  /** Creates a new AutoShooterAmpPos. */
  public AutoShooterAmpPos(ShooterAngle shootAngle, Shooter shoot, Processor process, Intake in, Double endingTime) {
    
    shooter = shoot;
    shooterAngle = shootAngle;
    processor = process;
    intake = in;
    endTime = endingTime;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!noteOnBoard) {

      // Run the shooter at the correct Amp Speeds
      shooter.runShooterAuto(TopShootAmpSpeed, BottomShootAmpSpeed);

      // Check to see if we are at the amp angle
      if ((Math.abs(CurrentShooterAngle - AmpAngle) < ShooterAngleTolerance) && shooterAngle.getTopSwitch() == false) {
        shooterAngle.runPivot(AngleMotorSpeed);
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

    shooter.runShooter(0);
    processor.runProcessor(0.0);
    intake.runIntake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     boolean doneYet = false;
    
    if(noteOnBoard)
    {
      doneYet = true;
    }
    if(timer.get() >= endTime-startTime)
    {
      doneYet = true;
    }
    if(killAuto == true )
    {
      doneYet = true;
    }

    return doneYet;
  }
}
