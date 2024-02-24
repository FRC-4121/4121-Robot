// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1NoteLeft extends SequentialCommandGroup {
  /** Creates a new Auto1NoteLeft. */
  public Auto1NoteLeft(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunShooterSpeaker(shoot, process, in, 2), new AutoDriveAndIntake(swerve, process, in, 0.1, 90.0, 10.0, 0.0, 10.0));
  }
}
