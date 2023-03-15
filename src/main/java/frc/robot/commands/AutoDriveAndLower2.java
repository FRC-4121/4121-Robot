// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveAndLower2 extends ParallelCommandGroup {
  /** Creates a new AutoDriveAndLower. */
  public AutoDriveAndLower2(SwerveDrive drive, NetworkTableQuerier table, ArmRotate armRotate, Pneumatics pneumatic, ArmExtend armExtend, Wrist wrist, double speed, double distance, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoArmTravelPos(armRotate,pneumatic,armExtend,wrist), 
    new AutoDrive(drive,speed,distance,angle,0,0,20,table));
  }
}
