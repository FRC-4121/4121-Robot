package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mutables;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.AutoAlignBase.ToMove;
import frc.robot.subsystems.SwerveDriveWPI;

/**
 * This class is creating for the various combined commands which that don't
 * really belong to a single subsystem
 */
public class CombinedCommands {
  private CombinedCommands() {

  }


  public static Command autoAlignBest(SwerveDriveWPI swerve, AutoAlignBase.Alignment align,
      NetworkTableQuerier.BestTag best, long[] filter) {
    return new AutoAlignBest(swerve, align, ToMove.RotateOnly, best, filter)
        .andThen(new AutoAlignBest(swerve, align, ToMove.DriveOnly, best, filter));
  }
}
