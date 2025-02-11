package frc.robot.commands;

import frc.robot.GlobalMutable;

/**
 * This further extends commands with a timeout to check if killAuto is set
 */
public class AutoCommand extends TimeoutCommand {
  public AutoCommand(double runTime) {
    super(runTime);
  }

  @Override
  public boolean isFinished() {
    return GlobalMutable.killAuto || super.isFinished();
  }
}
