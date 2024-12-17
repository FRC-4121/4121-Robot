// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterAngle;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
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
    wpiPIDController = new PIDController(kShooterAngleKP, kShooterAngleKI, kShooterAngleKD);
    wpiPIDController.setTolerance(1.0, 5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (AutoShooterPositioning) {

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
          if (blueAlliance) {
            if (ntable.getTagInfo("CAM2", i, "id") == BlueSpeakerCenterID) {
              closestTag = i;
              isMyTag = true;
            }
          } else {
            if (ntable.getTagInfo("CAM2", i, "id") == RedSpeakerCenterID) {
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

          if (tagDistance > MinAutoDistance && tagDistance < MaxAutoDistance) {
            if (blueAlliance) {
              switch (tagID) {
                case BlueAmpID:
                  ShooterTargetAngle = AmpAngle;
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = AmpEncoder;
                  LastShooterEncoder = AmpEncoder;
                  break;
                case BlueSpeakerCenterID:
                  ShooterTargetAngle = getTargetAngle(tagDistance);
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  LastShooterEncoder = ShooterTargetEncoder;
                  break;
                case BlueSpeakerSideID:
                  ShooterTargetAngle = getTargetAngle(tagDistance);
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  LastShooterEncoder = ShooterTargetEncoder;
                  break;
                default:
                  ShooterTargetAngle = LastShooterAngle;
                  ShooterTargetEncoder = LastShooterEncoder;
              }
            } else {
              switch (tagID) {
                case RedAmpID:
                  ShooterTargetAngle = AmpAngle;
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = AmpEncoder;
                  LastShooterEncoder = ShooterTargetEncoder;
                  break;
                case RedSpeakerCenterID:
                  ShooterTargetAngle = getTargetAngle(tagDistance);
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  LastShooterEncoder = ShooterTargetEncoder;
                  break;
                case RedSpeakerSideID:
                  ShooterTargetAngle = getTargetAngle(tagDistance);
                  LastShooterAngle = ShooterTargetAngle;
                  ShooterTargetEncoder = getTargetEncoder(tagDistance);
                  LastShooterEncoder = ShooterTargetEncoder;
                  break;
                default:
                  ShooterTargetAngle = LastShooterAngle;
                  ShooterTargetEncoder = LastShooterEncoder;
              }
            }
          } else {
            ShooterTargetAngle = LastShooterAngle;
            ShooterTargetEncoder = LastShooterEncoder;
          }
        } else {
          // ShooterTargetAngle = IdleAngle;
          ShooterTargetAngle = LastShooterAngle;
          LastShooterAngle = ShooterTargetAngle;
          ShooterTargetEncoder = LastShooterEncoder;
          LastShooterEncoder = ShooterTargetEncoder;
        }
      } else {
        // ShooterTargetAngle = IdleAngle;
        ShooterTargetAngle = LastShooterAngle;
        LastShooterAngle = ShooterTargetAngle;
        ShooterTargetEncoder = LastShooterEncoder;
        LastShooterEncoder = ShooterTargetEncoder;
      }

      // Determine new motor speed from PID controller
      double currentEncoder = shootAngle.getIntegratedValue();
      // double targetEncoder = getTargetEncoder(tagDistance);
      double pidOutput = -wpiPIDController.calculate(currentEncoder, ShooterTargetEncoder);

      SmartDashboard.putNumber("Target Encoder", ShooterTargetEncoder);

      // Limit PID putput
      if (pidOutput > 1) {
        pidOutput = 1;
      } else if (pidOutput < -1) {
        pidOutput = -1;
      }

      System.out.println("Target Angle:" + ShooterTargetAngle);
      System.out.println("PID Output:" + pidOutput);

      if (PauseAutoPosition == false) {
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
   * 
   * Calculate the target angle given the distance from target
   * 
   * @param distance Measured distance from target
   * 
   */
  public double getTargetAngle(double distance) {

    double calcAngle = ((LowSpeakerAngle - HighSpeakerAngle) / (MaxAutoDistance - MinAutoDistance))
        * (distance - MinAutoDistance) + HighSpeakerAngle;
    if (calcAngle > MaxSpeakerAngle) {
      calcAngle = MaxSpeakerAngle;
    } else if (calcAngle < MinSpeakerAngle) {
      calcAngle = MinSpeakerAngle;
    }
    SmartDashboard.putNumber("Target Angle:", calcAngle);
    return calcAngle;

  }

  /**
   * 
   * Calculate the target angle given the distance from target
   * 
   * @param distance Measured distance from target
   * 
   */
  public double getTargetEncoder(double distance) {

    double calcEncoder = -1.5585 * distance * distance + 618.65 * distance - 35584;
    return calcEncoder;

  }

}
