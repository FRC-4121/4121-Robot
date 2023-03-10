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
public class AutoDriveAndBalance extends SequentialCommandGroup {
  /** Creates a new AutoDriveAndBalance. */
  public AutoDriveAndBalance(SwerveDrive drive, NetworkTableQuerier table) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoDrive(drive,0.6,94,180,0,0,3,table)
      /*new AutoBalance(drive,0.45,180,0,0,20,table)
    new ParkCommand(drive)*/);
  }
}
