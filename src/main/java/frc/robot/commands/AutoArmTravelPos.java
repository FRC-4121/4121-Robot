// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArmTravelPos extends SequentialCommandGroup {
  /** Creates a new AutoArmTravelPos. */
  public AutoArmTravelPos(ArmRotate armRotate, Pneumatics pneumatic, ArmExtend armExtend, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoExtendArm(armExtend,ExtendStartLength,20),new AutoArmRotation(armRotate,RotateTravelAngle,20,pneumatic), new AutoArmHighPosParallel(wrist, armExtend));
  }
}