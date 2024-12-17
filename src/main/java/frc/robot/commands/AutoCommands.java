package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;

/**
 * This class is creating for the various autonomous commands which, despite how
 * things were previously, do *not* need their own types.
 */
public class AutoCommands {
    private AutoCommands() {
    }

    public static Command driveAndShoot(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in) {
        return new ParallelRaceGroup(new AutoDrive(swerve, 0.1, 100.0, 0.0, 0.0, 0.02, 13.0),
                new RunShooterSpeaker(shoot, process, in, 13));
    }

    public static Command driveAndIntake(SwerveDriveWPI swerve, Processor process, Intake in, double speed, double dist,
            double ang, double heading, double rotation, double endTime) {
        return new ParallelRaceGroup(new AutoDrive(swerve, speed, dist, ang, heading, rotation, 10.0),
                new TakeInNote(in, process, endTime));
    }

    public static Command pickupAndIntake(SwerveDriveWPI swerve, Intake in, Processor process,
            NetworkTableQuerier ntable) {
        return new ParallelRaceGroup(new AutoPickupNote(swerve, ntable, 0.05, 3.0), new TakeInNote(in, process, 3.0));
    }

    public static Command oneNoteCenter(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in) {
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                driveAndIntake(swerve, process, in, 0.1, 100.0, 0.0, 0.0, 0.02, 10.0));
    }

    public static Command oneNoteLeft(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in) {
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                driveAndIntake(swerve, process, in, 0.1, 80.0, 10.0, 0.0, 0.045, 10.0));
    }

    public static Command oneNoteRight(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in) {
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                driveAndIntake(swerve, process, in, 0.1, 90.0, 350.0, 0.0, -0.04, 10.0));
    }

    public static Command twoNoteCenter(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in,
            ShooterAngle angle, NetworkTableQuerier table) {
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                pickupAndIntake(swerve, in, process, table),
                new AutoShooterAngle(angle, 37, 1.5),
                new AutoRunShooterSpeaker(shoot, process, in, 2));
    }

    public static Command twoNoteLeft(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in,
            ShooterAngle angle, NetworkTableQuerier table) {
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                driveAndIntake(swerve, process, in, 0.1, 80.0, 10.0, 10.0, 0.0, 5.0),
                new AutoShooterAngle(angle, 37, 3),
                new AutoDrive(swerve, -0.1, 8.0, 10.0, 20.0, 0.0, 2.0),
                new AutoRunShooterSpeaker(shoot, process, in, 4));
    }

    /**
     * @deprecated This seems to have key functionality missing
     */
    @Deprecated
    public static Command twoNoteRight(SwerveDriveWPI swerve, Shooter shoot, Processor process, Intake in,
            ShooterAngle angle, NetworkTableQuerier table) {
        // I don't know why so much is commented out
        return new SequentialCommandGroup(new RunShooterSpeaker(shoot, process, in, 2),
                // new AutoDriveAndIntake(swerve,process,in,0.1, 90.0, 355, -10, 0.0, 10.0),
                // new AutoShooterAngle(angle,37,3),
                new AutoDrive(swerve, 0.2, 220.0, 307, 0.0, 0.0, 5.0));
        // new AutoRunShooterSpeaker(shoot, process, in, 4));
    }
}
