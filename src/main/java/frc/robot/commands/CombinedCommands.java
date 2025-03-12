package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return elevator.new PositionElevatorAndWait(ElevatorMM.ElevatorPositions.Load) // start by moving to the level
        .andThen(
            claw.autoRotate(CClaw.ClawPositions.Load).andThen(claw.intakeCoral()) // move the the load position and intake coral
                .deadlineFor(claw.new WithoutSafety().alongWith(Commands.idle(elevator)))); // while that's going, we want to disable safety
  }

  public static Command moveClaw(CClaw claw, ElevatorMM elevator, double elevatorPos, double clawPos) {
    return elevator.new PositionElevatorAndWait(elevatorPos)
        .alongWith(
            claw.new RotateClawAndWait(clawPos))
        .withDeadline(Commands.waitSeconds(2.0));
  }
}
