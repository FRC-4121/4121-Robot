package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mutables;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.AutoAlignBase.ToMove;
import frc.robot.subsystems.CClaw;
import frc.robot.subsystems.ElevatorMM;
import frc.robot.subsystems.SwerveDriveWPI;

/**
 * This class is creating for the various combined commands which that don't
 * really belong to a single subsystem
 */
public class CombinedCommands {
  private CombinedCommands() {

  }

  public static Command combinedLoad(CClaw claw, ElevatorMM elevator) {
    return claw.new RotateClawAndWait(CClaw.ClawPositions.Home)
        .andThen(Commands.idle(claw, elevator).until(() -> Mutables.isClawClear))
        .andThen(elevator.new PositionElevatorAndWait(ElevatorMM.ElevatorPositions.Load))
        .andThen(
            claw.autoRotate(CClaw.ClawPositions.Load).andThen(claw.intakeCoral())
                .deadlineFor(claw.new WithoutSafety().alongWith(Commands.idle(elevator))));
  }

  public static Command moveClaw(CClaw claw, ElevatorMM elevator, double elevatorPos, double clawPos) {
    return claw.new RotateClawAndWait(CClaw.ClawPositions.Home)
        .andThen(elevator.new PositionElevatorAndWait(elevatorPos))
        .andThen(
            claw.new RotateClawAndWait(clawPos))
        .withDeadline(Commands.waitSeconds(2.0));
  }

  public static Command shootCoral(CClaw claw, ElevatorMM elevator) {
    return claw.scoreCoral().andThen(claw.new RotateClawAndWait(CClaw.ClawPositions.Home))
        .andThen(elevator.positionElevator(ElevatorMM.ElevatorPositions.Load));
  }

  public static Command autoAlignBest(SwerveDriveWPI swerve, AutoAlignBase.Alignment align,
      NetworkTableQuerier.BestTag best, long[] filter) {
    return new AutoAlignBest(swerve, align, ToMove.RotateOnly, best, filter)
        .andThen(new AutoAlignBest(swerve, align, ToMove.DriveOnly, best, filter));
  }
}
