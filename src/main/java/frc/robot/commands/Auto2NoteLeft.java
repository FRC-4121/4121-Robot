// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2NoteLeft extends SequentialCommandGroup {
  /** Creates a new Auto2NoteLeft. */
  /** Creates a new Auto2NoteCenter. */
  public Auto2NoteLeft(SwerveDriveWPI swerve, Shooter shoot, Processor process, ShooterAngle angle, Intake in) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunShooterSpeaker(shoot, process, in, 2), 
    new AutoDriveAndIntake(swerve,process,in,0.1, 80.0, 10.0, 10.0, 0.0, 5.0),
    new AutoShooterAngle(angle,37,3),
    new AutoDrive(swerve,-0.1, 8.0, 10.0, 20.0, 0.0, 2.0),
    new AutoRunShooterSpeaker(shoot, process, in, 4));
  }
}
