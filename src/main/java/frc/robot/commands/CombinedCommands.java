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
    return elevator.new PositionElevatorAndWait(ElevatorMM.ElevatorPositions.LOAD) // start by moving to the level
        .andThen(
            claw.autoRotate(CClaw.ClawPositions.Load).andThen(claw.intakeCoral()) // move the the load position and intake coral
                .deadlineFor(claw.new WithoutSafety())) // while that's going, we want to disable safety
        .withDeadline(Commands.waitSeconds(5.0)); // don't let it go for more than 5 seconds
  }

  public static Command prepL1(CClaw claw, ElevatorMM elevator) {
    return elevator.new PositionElevatorAndWait(ElevatorMM.ElevatorPositions.CORAL1)
        .alongWith(
            claw.new RotateClawAndWait(CClaw.ClawPositions.L1Score))
        .withDeadline(Commands.waitSeconds(2.0));
  }
}
