// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.MechanismConstants.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.*;

public class AutoShooterPos extends Command {

  private ShooterAngle shootAngle;
  private NetworkTableQuerier ntable;
  private double startTime;
  private double stopTime;
  private Timer timer;

  private double targetAngle;
  private double angleTolerance;

  private PIDController wpiPIDController;
 
  /** Creates a new AutoShooterPos. */
  public AutoShooterPos(ShooterAngle angle, NetworkTableQuerier table, double endTime) {

    // Set local variables
    shootAngle = angle;
    ntable = table;
    stopTime = endTime;

    addRequirements(shootAngle);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start timer and get current time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

    // Initialize variables
    targetAngle = 45.0;
    angleTolerance = 1.0;

    // Create PID controller
    wpiPIDController = new PIDController(kShooterAngleKP, kShooterAngleKI, kShooterAngleKD);
    wpiPIDController.setTolerance(1.0,5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the distance to the AprilTag (if we see an AprilTag)

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop the shooter angle motor
    shootAngle.runPivot(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean thereYet = false;

    double time = timer.get();

    if (Math.Abs(CurrentShooterAngle - targetAngle) <= angleTolerance) {
      thereYet = true;
    }
    else if (shootAngle.getTopSwitch() == true) {
      thereYet = true;
    }
    else if (shootAngle.getBottomSwitch() == true) {
      thereYet = true;
    }
    else if (stopTime <= time - startTime) {
      thereYet = true;
    }
    else if (killAuto == true)
    {
      thereYet = true;
    }

    return thereYet;

  }

  // Calculate the target angle based on vision distance
  public double GetTargetAngle(double distance) {

    double calcAngle = ((HighSpeakerAngle - LowSpeakerAngle)/(MaxAutoDistance - MinAutoDistance)) * (distance - MinAutoDistance) + LowSpeakerAngle;
    return calcAngle;

  }

}
