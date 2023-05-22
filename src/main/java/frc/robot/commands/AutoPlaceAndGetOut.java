// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceAndGetOut extends SequentialCommandGroup {
  /** Creates a new AutoPlaceAndGetOut. */
  public AutoPlaceAndGetOut(SwerveDrive drive, NetworkTableQuerier table, ArmRotate armRotate, Pneumatics pneumatic, ArmExtend armExtend, Wrist wrist, Grabber grab) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     addCommands(new AutoArmHighCube(armRotate,pneumatic,armExtend,wrist, grab), 
     new AutoDriveAndLower(drive,table,armRotate,pneumatic,armExtend,wrist,0.7,160,180));
    // addCommands(
    //  new AutoDriveAndLower(drive,table,armRotate,pneumatic,armExtend,wrist,0.7,160,180));
    //addCommands(new AutoArmHighCone(armRotate,pneumatic,armExtend,wrist, grab));
  }
}
