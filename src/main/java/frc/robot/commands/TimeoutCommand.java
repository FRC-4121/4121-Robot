package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command that that completes after a given timeout.
 */
public class TimeoutCommand extends Command {
    private Timer timer;
    private double runTime;

    public TimeoutCommand(double runTime) {
        this.runTime = runTime;
        this.timer = new Timer();
    }

    protected double currentTime() {
        return timer.get();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > runTime;
    }
}
