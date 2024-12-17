// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoShooterAngle extends AutoCommand {
  private ShooterAngle shootAngle;

  private double targetEncoder;

  private PIDController wpiPIDController;

  /** Creates a new AutoShooterPos. */
  public AutoShooterAngle(ShooterAngle shootAngle, double targetEncoder, double time) {
    super(time);
    // Set local variables
    this.shootAngle = shootAngle;
    this.targetEncoder = targetEncoder;
    addRequirements(shootAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    // Create PID controller
    wpiPIDController = new PIDController(kShooterAngleKP, kShooterAngleKI, kShooterAngleKD);
    wpiPIDController.setTolerance(1.0, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentEncoder = shootAngle.getIntegratedValue();

    // Determine new motor speed from PID controller
    double pidOutput = -wpiPIDController.calculate(currentEncoder, targetEncoder);
    if (pidOutput > 1) {
      pidOutput = 1;
    } else if (pidOutput < -1) {
      pidOutput = -1;
    }

    // Run angle motor at new speed (as long as we aren't at bounds)
    if (Math.abs(currentEncoder - ShooterTargetEncoder) > ShooterAngleTolerance) {

      double angleSpeed = AngleMotorSpeed * pidOutput;

      if (pidOutput > 0) {

        if (shootAngle.getTopSwitch() == false) {

          shootAngle.runPivot(AngleMotorMinSpeed + angleSpeed);
          SmartDashboard.putNumber("Angle Input", AngleMotorMinSpeed + angleSpeed);

        } else {

          shootAngle.runPivot(0);

        }

      } else if (pidOutput < 0) {

        if (shootAngle.getBottomSwitch() == false) {

          shootAngle.runPivot(-AngleMotorMinSpeed + angleSpeed);
          SmartDashboard.putNumber("Angle Input", -AngleMotorMinSpeed + angleSpeed);

        } else {

          shootAngle.runPivot(0);

        }

      }

      readyToShoot = false;

    } else {

      shootAngle.runPivot(0.0);
      readyToShoot = true;

    }

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
    if (super.isFinished())
      return true;
    return (Math.abs(shootAngle.getIntegratedValue() - ShooterTargetEncoder) < ShooterAngleTolerance)
        || shootAngle.getBottomSwitch();
  }
}
