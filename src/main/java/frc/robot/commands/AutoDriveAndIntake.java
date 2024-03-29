// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveAndIntake extends ParallelRaceGroup {
  /** Creates a new AutoDriveBackAndIntake. */
  public AutoDriveAndIntake(SwerveDriveWPI swerve, Processor process, Intake in, double speed, double dist, double ang, double heading, double rotation, double endTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoDrive(swerve, speed, dist, ang, heading, rotation, 10.0), new TakeInNote(in, process, endTime));
  }
}
