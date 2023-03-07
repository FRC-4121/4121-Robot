// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArmHighPos extends ParallelCommandGroup {
  /** Creates a new AutoArmHighPos. */
  public AutoArmHighPos(ArmRotate armRotate, Pneumatics pneumatic, ArmExtend armExtend) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoArmRotation(armRotate,RotateHighAngle,20,pneumatic), new AutoExtendArm(armExtend,ExtendHighLength,20));
    //addCommands(new AutoArmRotation(armRotate,RotateHighAngle,20,pneumatic));
    //addCommands(new AutoExtendArm(armExtend,ExtendHighLength,20));
  }
}
