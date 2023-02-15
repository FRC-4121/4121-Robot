// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;


import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoDrive extends CommandBase {

  /** Creates a new AutoDrive. */
  private final SwerveDrive drivetrain;
  private double targetDriveDistance; //inches
  private double targetAngle;
  private double targetRotation;
  private double targetSpeed;
  private double direction;
  private double stopTime;
  private double currentGyroAngle = 0;
  private double driveDirection;
  private double driveSpeed;

  private double angleCorrection, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double leftFrontEncoderStart;
  private double rightFrontEncoderStart;
  private double leftBackEncoderStart;
  private double rightBackEncoderStart;

  private NetworkTableQuerier ntables;

  private double totalRotationsLeft = 0;
  private double totalRotationsRight = 0;

  private Timer timer = new Timer();
  private PIDController pidDriveAngle;
  private PIDController pidDriveDistance; 

  //distance is in inches

  public AutoDrive(SwerveDrive drive, double speed, double dis, double ang, double rot, double time, NetworkTableQuerier table) {
   
    targetSpeed = speed;
    targetDriveDistance = dis;
    targetAngle = ang;
    targetRotation = rot;
    stopTime = time;
    ntables = table;
    drivetrain = drive;

    addRequirements(drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    distanceTraveled = 0.0;
    timer.start();
    startTime = timer.get();

    // Zero the gyro and encoders
    //drivetrain.zeroGyro();
    drivetrain.zeroEncoders();

    ntables.zeroPiGyro();

    leftFrontEncoderStart = drivetrain.getLeftFrontDriveEncoder();
    rightFrontEncoderStart = drivetrain.getRightFrontDriveEncoder();
    leftBackEncoderStart = drivetrain.getLeftBackDriveEncoder();
    rightBackEncoderStart = drivetrain.getRightBackDriveEncoder();

    angleCorrection = 0;
    speedCorrection = 1;

    //The constants for these need to be figured out
    pidDriveAngle = new PIDController(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
    pidDriveDistance = new PIDController(kP_Straight, kI_Straight, kD_Straight);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate angle correction based on gyro reading
    currentGyroAngle = ntables.getPiGyro();

    SmartDashboard.putNumber("DriveSpeed", targetSpeed);
    // Enforce minimum speed
    // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {
    //   angleCorrection = 0;
    //   if (driveSpeed < 0){
    //     driveSpeed = -kAutoDriveSpeedMin;
    //   } else {
    //     driveSpeed = kAutoDriveSpeedMin;
    //   }
    // }

    double leftStickY = -targetSpeed * Math.cos(Math.toRadians(targetAngle));
    double leftStickX = targetSpeed * Math.sin(Math.toRadians(targetAngle));
    SmartDashboard.putNumber("Left Stick X", leftStickX);
    SmartDashboard.putNumber("Left Stick Y", leftStickY);


    // Run the drive
    drivetrain.drive(leftStickX, leftStickY, targetRotation);

    SmartDashboard.putNumber("Left Front Start", leftFrontEncoderStart);
    SmartDashboard.putNumber("Right Front Start", rightFrontEncoderStart);
    SmartDashboard.putNumber("Left Back Start", leftBackEncoderStart);
    SmartDashboard.putNumber("Right Back Start", rightBackEncoderStart);

    SmartDashboard.putNumber("Left Front Now", drivetrain.getLeftFrontDriveEncoder());
    SmartDashboard.putNumber("Right Front Now", drivetrain.getRightFrontDriveEncoder());
    SmartDashboard.putNumber("Left Back Now", drivetrain.getLeftBackDriveEncoder());
    SmartDashboard.putNumber("Right Back Now", drivetrain.getRightBackDriveEncoder());    

    // calculate driven distance
    double totalRotationsLeftFront = Math.abs((drivetrain.getLeftFrontDriveEncoder() - leftFrontEncoderStart)); //does getEncoderPosition return rotations or another unit?
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsLeftFront) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Left Front Rotations", distanceTraveled);

    double totalRotationsRightFront = Math.abs((drivetrain.getRightFrontDriveEncoder() - rightFrontEncoderStart));
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsRightFront) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Right Front Rotations", distanceTraveled);

    double totalRotationsLeftBack = Math.abs((drivetrain.getLeftBackDriveEncoder() - leftBackEncoderStart));
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsLeftBack) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Left Back Rotations", distanceTraveled);

    double totalRotationsRightBack = Math.abs((drivetrain.getRightBackDriveEncoder() - rightBackEncoderStart));
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsRightBack) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Right Back Rotations", distanceTraveled);

    double averageRotations = ((totalRotationsLeftFront + totalRotationsRightFront + totalRotationsLeftBack + totalRotationsRightBack) / 4.0);
    distanceTraveled = (kWheelDiameter * Math.PI * averageRotations) / (kTalonFXPPR * kDriveGearRatio);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
    distanceTraveled = 0.0;

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    double time = timer.get();

    // Check distance against target
    SmartDashboard.putNumber("Distance error", Math.abs(distanceTraveled - targetDriveDistance));
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
    SmartDashboard.putNumber("Target Distance", targetDriveDistance);
    if (distanceTraveled >= targetDriveDistance) {
       thereYet = true;
    } else if (time - startTime >= stopTime) {
      thereYet = true;
    } else if (killAuto == true) {
      thereYet = true;
    }

    return thereYet;
  }

}

