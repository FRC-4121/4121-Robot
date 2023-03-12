// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArmLoadPosParallel extends ParallelCommandGroup {
  /** Creates a new AutoArmLoadPosParallel. */
  public AutoArmLoadPosParallel(Wrist wrist, ArmExtend armExtend) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoMoveWrist(wrist,WristLoadPosition,5), new AutoExtendArm(armExtend,ExtendLoadLength,20));
  }
}
