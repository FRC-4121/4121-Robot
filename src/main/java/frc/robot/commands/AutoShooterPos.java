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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;

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

  private MedianFilter filter; 
 
  /** Creates a new AutoShooterPos. */
  public AutoShooterPos(ShooterAngle angle, NetworkTableQuerier table) {

    // Set local variables
    shootAngle = angle;
    ntable = table;

    filter = new MedianFilter(20);

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

    if(AutoShooterPositioning) {
    
    System.out.println("Started Auto Shooter Angle");

    // Determine if we see any AprilTags
    tagsFound = ntable.getTagsFound("CAM2");
    System.out.println("Tags Found:"+tagsFound);

    // If AprilTags found, determine which ones and set target angle
    // If no AprilTags found, set target angle to IDLE position
    if (tagsFound > 0) {

      // Find the closest tag
      for (int i = 0; i < tagsFound; i++) {

        if (blueAlliance) {
          if (ntable.getTagInfo("Cam2", i, "id") == BlueSpeakerCenterID) {

            closestTag = i;

          }
        } else {
          if (ntable.getTagInfo("Cam2", i, "id") == RedSpeakerCenterID) {

            closestTag = i;

          }
        }
      }

      // Get ID and distance for closest tag
      tagID = (int)ntable.getTagInfo("CAM2", closestTag, "id");
      tagDistance = filter.calculate(ntable.getTagInfo("CAM2", closestTag, "distance"));
      System.out.println("Tag Distance:"+tagDistance);
      System.out.println("TagID:"+tagID);
  
      System.out.println("Blue Alliance:"+blueAlliance);

      // Determine if the tag belongs to my alliance
      if (blueAlliance && tagID == BlueSpeakerCenterID) {
        isMyTag = true;
      }
      else if (!blueAlliance && tagID == RedSpeakerCenterID) {
        isMyTag = true;
      }

      if (isMyTag) {
        System.out.println("Auto Angle Tag Found");
      }

      // Set target angle based on which AprilTag was found for my alliance
      if (tagDistance > MinAutoDistance && tagDistance < MaxAutoDistance) {
        System.out.println("Auto Angle Adjust");
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

        ShooterTargetAngle = IdleAngle;
        //ShooterTargetAngle = LastShooterAngle;
        LastShooterAngle = ShooterTargetAngle;
      }

    } else {

      ShooterTargetAngle = IdleAngle;
      //ShooterTargetAngle = LastShooterAngle;
      LastShooterAngle = ShooterTargetAngle;

    }

    // Determine new motor speed from PID controller
    double pidOutput = wpiPIDController.calculate(CurrentShooterAngle, ShooterTargetAngle);
    if(pidOutput > 1)
    {
      pidOutput = 1;
    } else if (pidOutput < -1)
    {
      pidOutput = -1;
    }

    System.out.println("Target Angle:" + ShooterTargetAngle);
    System.out.println("PID Output:" + pidOutput);

    if(PauseAutoPosition == false) {
    // Run angle motor at new speed (as long as we aren't at bounds)
    if (Math.abs(CurrentShooterAngle - ShooterTargetAngle) > ShooterAngleTolerance) {

      double angleSpeed = AngleMotorSpeed * pidOutput;

      if (pidOutput > 0) {

        if (shootAngle.getTopSwitch() == false) {
          shootAngle.runPivot(AngleMotorMinSpeed + angleSpeed);
          SmartDashboard.putNumber("Angle Input", AngleMotorMinSpeed + angleSpeed);
        } else {
          shootAngle.runPivot(0);
        }

      }
      else if (pidOutput < 0) {

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

      System.out.println("Angle Motor Speed: 0.0");

    }
  }}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("Auto Shooter Angle Stopped");
    // Stop the shooter angle motor
    shootAngle.runPivot(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean thereYet = false;

    if (killAuto) {
      thereYet = true;
    }

    return thereYet;

  }

  // Calculate the target angle based on vision distance
  public double getTargetAngle(double distance) {

    double calcAngle = ((LowSpeakerAngle - HighSpeakerAngle)/(MaxAutoDistance - MinAutoDistance)) * (distance - MinAutoDistance) + HighSpeakerAngle;
    if(calcAngle > MaxSpeakerAngle){
      calcAngle = MaxSpeakerAngle;
    } else if(calcAngle < MinSpeakerAngle){
      calcAngle = MinSpeakerAngle;
    }
    SmartDashboard.putNumber("Target Angle:",calcAngle);
    return calcAngle;

  }

}
