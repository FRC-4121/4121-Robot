package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mutables;
import frc.robot.subsystems.CClaw;
import frc.robot.subsystems.ElevatorMM;

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
        .andThen(elevator.new PositionElevatorAndWait(ElevatorMM.ElevatorPositions.Load)) // start by moving to the
                                                                                          // level
        .andThen(
            claw.autoRotate(CClaw.ClawPositions.Load).andThen(claw.intakeCoral()) // move the the load position and
                                                                                  // intake coral
                .deadlineFor(claw.new WithoutSafety().alongWith(Commands.idle(elevator)))); // while that's going, we
                                                                                            // want to disable safety
  }

  public static Command moveClaw(CClaw claw, ElevatorMM elevator, double elevatorPos, double clawPos) {
    return claw.new RotateClawAndWait(CClaw.ClawPositions.Home - 0.15)
        .andThen(elevator.new PositionElevatorAndWait(elevatorPos))
        .andThen(
            claw.new RotateClawAndWait(clawPos))
        .withDeadline(Commands.waitSeconds(2.0));
  }

  public static Command shootCoral(CClaw claw, ElevatorMM elevator) {
    return claw.scoreCoral().andThen(claw.new RotateClawAndWait(CClaw.ClawPositions.Home))
        .andThen(elevator.positionElevator(ElevatorMM.ElevatorPositions.Load));
  }
}
