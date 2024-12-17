package frc.robot.commands;

import static frc.robot.Constants.killAuto;

/**
 * This further extends commands with a timeout to check if killAuto is set
 */
public class AutoCommand extends TimeoutCommand {
    public AutoCommand(double runTime) {
        super(runTime);
    }
    @Override
    public boolean isFinished() {
        return killAuto || super.isFinished();
    }
}
