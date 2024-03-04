// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2NoteCenter extends SequentialCommandGroup {
  /** Creates a new Auto2NoteCenter. */
  public Auto2NoteCenter(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in, ShooterAngle angle, NetworkTableQuerier table) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunShooterSpeaker(shoot, process, in, 2), 
    new AutoPickupAndIntake(swerve, in, process, table),
    new AutoShooterAngle(angle,37,1.5), 
    new AutoRunShooterSpeaker(shoot, process, in, 2));
  }
  //new AutoDriveAndIntake(swerve,process,in,0.2, 100.0, 0.0, 0.0, 0.02, 10.0),
  //new AutoPickupAndIntake(swerve, in, process, table)
}
