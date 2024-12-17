// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterAngle;
import frc.robot.Constants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;

public class AutoShooterPos extends AutoCommand {

  private ShooterAngle shootAngle;
  private NetworkTableQuerier ntable;

  private double tagsFound;
  private int closestTag;
  private double closestDistance;
  private int tagID;
  private double tagDistance;
  private boolean isMyTag;
  private double lastShooterAngle = 40;
  private double lastShooterEncoder = 12000;

  private PIDController wpiPIDController;

  private MedianFilter filter;

  /** Creates a new AutoShooterPos. */
  public AutoShooterPos(ShooterAngle angle, NetworkTableQuerier table) {
    super(60.0); // The old version didn't have a timeout
    // Set local variables
    shootAngle = angle;
    ntable = table;

    filter = new MedianFilter(20);

    addRequirements(shootAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    // Initialize variables
    tagsFound = 0;
    closestTag = 0;
    tagID = 0;
    closestDistance = 100000;// Really large because no rings will ever be that far
    tagDistance = 0;
    isMyTag = false;

    // Create PID controller
    wpiPIDController = new PIDController(MechanismConstants.kShooterAngleKP, MechanismConstants.kShooterAngleKI,
        MechanismConstants.kShooterAngleKD);
    wpiPIDController.setTolerance(1.0, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (MechanismConstants.AutoShooterPositioning) {

      System.out.println("Started Auto Shooter Angle");

      // Initialize variables
      tagDistance = 0.0;
      isMyTag = false;

      // Determine if we see any AprilTags
      tagsFound = ntable.getTagsFound("CAM2");

      // If AprilTags found, determine which ones and set target angle
      // If no AprilTags found, set target angle to IDLE position
      closestTag = -1;
      if (tagsFound > 0) {
        // Find the closest tag
        for (int i = 0; i < tagsFound; i++) {
          if (Constants.blueAlliance) {
            if (ntable.getTagInfo("CAM2", i, "id") == MechanismConstants.BlueSpeakerCenterID) {
              closestTag = i;
              isMyTag = true;
            }
          } else {
            if (ntable.getTagInfo("CAM2", i, "id") == MechanismConstants.RedSpeakerCenterID) {
              closestTag = i;
              isMyTag = true;
            }
          }
        }

        // If proper speaker tag found, proceed with auto angle
        if (isMyTag) {
          // Get tags distance and determine proper action
          tagID = (int) ntable.getTagInfo("CAM2", closestTag, "id");
          tagDistance = filter.calculate(ntable.getTagInfo("CAM2", closestTag, "distance"));

          if (tagDistance > MechanismConstants.MinAutoDistance && tagDistance < MechanismConstants.MaxAutoDistance) {
            if (Constants.blueAlliance) {
              switch (tagID) {
                case MechanismConstants.BlueAmpID:
                  MechanismConstants.ShooterTargetAngle = MechanismConstants.AmpAngle;
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = MechanismConstants.AmpEncoder;
                  lastShooterEncoder = MechanismConstants.AmpEncoder;
                  break;
                case MechanismConstants.BlueSpeakerCenterID:
                  MechanismConstants.ShooterTargetAngle = getTargetAngle(tagDistance);
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
                  break;
                case MechanismConstants.BlueSpeakerSideID:
                  MechanismConstants.ShooterTargetAngle = getTargetAngle(tagDistance);
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
                  break;
                default:
                  MechanismConstants.ShooterTargetAngle = lastShooterAngle;
                  MechanismConstants.ShooterTargetEncoder = lastShooterEncoder;
              }
            } else {
              switch (tagID) {
                case MechanismConstants.RedAmpID:
                  MechanismConstants.ShooterTargetAngle = MechanismConstants.AmpAngle;
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = MechanismConstants.AmpEncoder;
                  lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
                  break;
                case MechanismConstants.RedSpeakerCenterID:
                  MechanismConstants.ShooterTargetAngle = getTargetAngle(tagDistance);
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
                  break;
                case MechanismConstants.RedSpeakerSideID:
                  MechanismConstants.ShooterTargetAngle = getTargetAngle(tagDistance);
                  lastShooterAngle = MechanismConstants.ShooterTargetAngle;
                  MechanismConstants.ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
                  break;
                default:
                  MechanismConstants.ShooterTargetAngle = lastShooterAngle;
                  MechanismConstants.ShooterTargetEncoder = lastShooterEncoder;
              }
            }
          } else {
            MechanismConstants.ShooterTargetAngle = lastShooterAngle;
            MechanismConstants.ShooterTargetEncoder = lastShooterEncoder;
          }
        } else {
          // ShooterTargetAngle = IdleAngle;
          MechanismConstants.ShooterTargetAngle = lastShooterAngle;
          lastShooterAngle = MechanismConstants.ShooterTargetAngle;
          MechanismConstants.ShooterTargetEncoder = lastShooterEncoder;
          lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
        }
      } else {
        // ShooterTargetAngle = IdleAngle;
        MechanismConstants.ShooterTargetAngle = lastShooterAngle;
        lastShooterAngle = MechanismConstants.ShooterTargetAngle;
        MechanismConstants.ShooterTargetEncoder = lastShooterEncoder;
        lastShooterEncoder = MechanismConstants.ShooterTargetEncoder;
      }

      // Determine new motor speed from PID controller
      double currentEncoder = shootAngle.getIntegratedValue();
      // double targetEncoder = getTargetEncoder(tagDistance);
      double pidOutput = -wpiPIDController.calculate(currentEncoder, MechanismConstants.ShooterTargetEncoder);

      SmartDashboard.putNumber("Target Encoder", MechanismConstants.ShooterTargetEncoder);

      // Limit PID putput
      if (pidOutput > 1) {
        pidOutput = 1;
      } else if (pidOutput < -1) {
        pidOutput = -1;
      }

      System.out.println("Target Angle:" + MechanismConstants.ShooterTargetAngle);
      System.out.println("PID Output:" + pidOutput);

      if (MechanismConstants.PauseAutoPosition == false) {
        // Run angle motor at new speed (as long as we aren't at bounds)
        if (Math
            .abs(currentEncoder - MechanismConstants.ShooterTargetEncoder) > MechanismConstants.ShooterAngleTolerance) {
          double angleSpeed = MechanismConstants.AngleMotorSpeed * pidOutput;
          if (pidOutput > 0) {
            if (shootAngle.getTopSwitch() == false) {
              shootAngle.runPivot(MechanismConstants.AngleMotorMinSpeed + angleSpeed);
              SmartDashboard.putNumber("Angle Input", MechanismConstants.AngleMotorMinSpeed + angleSpeed);
            } else {
              shootAngle.runPivot(0);
            }
          } else if (pidOutput < 0) {
            if (shootAngle.getBottomSwitch() == false) {
              shootAngle.runPivot(-MechanismConstants.AngleMotorMinSpeed + angleSpeed);
              SmartDashboard.putNumber("Angle Input", -MechanismConstants.AngleMotorMinSpeed + angleSpeed);
            } else {
              shootAngle.runPivot(0);
            }
          }
          Constants.readyToShoot = false;
        } else {
          shootAngle.runPivot(0.0);
          Constants.readyToShoot = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("Auto Shooter Angle Stopped");
    // Stop the shooter angle motor
    shootAngle.runPivot(0);

  }

  /**
   * Calculate the target angle given the distance from target
   * 
   * @param distance Measured distance from target
   */
  public double getTargetAngle(double distance) {
    double calcAngle = ((MechanismConstants.LowSpeakerAngle - MechanismConstants.HighSpeakerAngle)
        / (MechanismConstants.MaxAutoDistance - MechanismConstants.MinAutoDistance))
        * (distance - MechanismConstants.MinAutoDistance) + MechanismConstants.HighSpeakerAngle;
    if (calcAngle > MechanismConstants.MaxSpeakerAngle) {
      calcAngle = MechanismConstants.MaxSpeakerAngle;
    } else if (calcAngle < MechanismConstants.MinSpeakerAngle) {
      calcAngle = MechanismConstants.MinSpeakerAngle;
    }
    SmartDashboard.putNumber("Target Angle:", calcAngle);
    return calcAngle;
  }

  /**
   * Calculate the target angle given the distance from target
   * 
   * @param distance Measured distance from target
   */
  public double getTargetEncoder(double distance) {
    double calcEncoder = -1.5585 * distance * distance + 618.65 * distance - 35584;
    return calcEncoder;
  }

  public double getLastAngle() {
    return lastShooterAngle;
  }
}
