// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.ExtraClasses.NetworkTableQuerier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup1 extends SequentialCommandGroup {
  /** Creates a new AutoGroup1. */
  public AutoGroup1(SwerveDrive swerve, NetworkTableQuerier table) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Distance compounds after each command
    addCommands(new AutoDrive(swerve, 0.6, 84, 0, 0,0, 5,table), new AutoDrive(swerve, 0.6, 120, 45, 0,0, 5,table),new AutoDrive(swerve, 0.6, 150, 270, 0,0, 5,table));
  }
}
