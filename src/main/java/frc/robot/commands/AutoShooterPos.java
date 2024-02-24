// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.math.controller.*;

public class AutoShooterPos extends Command {

  private ShooterAngle shootAngle;
  private NetworkTableQuerier ntable;

  private double tagsFound;
  private int closestTag;
  private double closestDistance;
  private int tagID;
  private double tagDistance;
  private boolean isMyTag;

  private PIDController wpiPIDController;
 
  /** Creates a new AutoShooterPos. */
  public AutoShooterPos(ShooterAngle angle, NetworkTableQuerier table) {

    // Set local variables
    shootAngle = angle;
    ntable = table;

    addRequirements(shootAngle);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Initialize variables
    tagsFound = 0;
    closestTag = 0;
    tagID = 0;
    closestDistance = 100000;//Really large because no rings will ever be that far
    tagDistance = 0;
    isMyTag = false;

    // Create PID controller
    wpiPIDController = new PIDController(kShooterAngleKP, kShooterAngleKI, kShooterAngleKD);
    wpiPIDController.setTolerance(1.0,5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Determine if we see any AprilTags
    tagsFound = ntable.getTagsFound("Cam2");

    // If AprilTags found, determine which ones and set target angle
    // If no AprilTags found, set target angle to IDLE position
    if (tagsFound > 0) {

      // Find the closest tag
      for (int i = 0; i < tagsFound; i++) {

        if (ntable.getTagInfo("Cam2",i,"distance") < closestDistance) {

          closestDistance = ntable.getTagInfo("Cam2",i,"distance");
          closestTag = i;

        }

      }

      // Get ID and distance for closest tag
      tagID = (int)ntable.getTagInfo("Cam2", closestTag, "id");
      tagDistance = ntable.getTagInfo("Cam2", closestTag, "distance");

      // Determine if the tag belongs to my alliance
      if (blueAlliance && (tagID == BlueAmpID || tagID == BlueSpeakerCenterID || tagID == BlueSpeakerSideID)) {
        isMyTag = true;
      }
      else if (!blueAlliance && (tagID == RedAmpID || tagID == RedSpeakerCenterID || tagID == RedSpeakerSideID)) {
        isMyTag = true;
      }

      // Set target angle based on which AprilTag was found for my alliance
      if (isMyTag) {

        if (blueAlliance) {

          switch (tagID) {

            case BlueAmpID:
              ShooterTargetAngle = AmpAngle;
              LastShooterAngle = ShooterTargetAngle;
              break;

            case BlueSpeakerCenterID:
              ShooterTargetAngle = getTargetAngle(tagDistance);
              LastShooterAngle = ShooterTargetAngle;
              break;

            case BlueSpeakerSideID:
              ShooterTargetAngle = getTargetAngle(tagDistance);
              LastShooterAngle = ShooterTargetAngle;
              break;

            default:
              ShooterTargetAngle = LastShooterAngle;


          }
  
        } else {

          switch (tagID) {

            case RedAmpID:
              ShooterTargetAngle = AmpAngle;
              LastShooterAngle = ShooterTargetAngle;
              break;

            case RedSpeakerCenterID:
              ShooterTargetAngle = getTargetAngle(tagDistance);
              LastShooterAngle = ShooterTargetAngle;
              break;

            case RedSpeakerSideID:
              ShooterTargetAngle = getTargetAngle(tagDistance);
              LastShooterAngle = ShooterTargetAngle;
              break;

            default:
              ShooterTargetAngle = LastShooterAngle;
              
          }

        }

      } else {

        ShooterTargetAngle = LastShooterAngle;

      }

    } else {

      ShooterTargetAngle = HighSpeakerAngle;
      LastShooterAngle = ShooterTargetAngle;

    }

    // Determine new motor speed from PID controller
    double pidOutput = wpiPIDController.calculate(CurrentShooterAngle, ShooterTargetAngle);

    // Run angle motor at new speed (as long as we aren't at bounds)
    if (Math.abs(CurrentShooterAngle - ShooterTargetAngle) > ShooterAngleTolerance) {

      if (pidOutput > 0 && shootAngle.getTopSwitch() == false) {

        shootAngle.runPivot(AngleMotorSpeed * pidOutput);

      }
      else if (pidOutput < 0 && shootAngle.getBottomSwitch() == false) {

        shootAngle.runPivot(AngleMotorSpeed * pidOutput);

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

    boolean thereYet = false;

    if (killAuto) {
      thereyet = true;
    }

    return thereYet;

  }

  // Calculate the target angle based on vision distance
  public double getTargetAngle(double distance) {

    double calcAngle = ((HighSpeakerAngle - LowSpeakerAngle)/(MaxAutoDistance - MinAutoDistance)) * (distance - MinAutoDistance) + LowSpeakerAngle;
    return calcAngle;

  }

}
