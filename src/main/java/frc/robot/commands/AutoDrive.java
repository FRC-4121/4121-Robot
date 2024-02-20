// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;


public class AutoDrive extends Command {

  /** Creates a new AutoDrive. */
  private final SwerveDrive drivetrain;
  private double targetDriveDistance; //inches
  private double targetAngle;
  private double targetRotation;
  private double chassisRotation;
  private double targetSpeed;
  private double stopTime;
  private double currentGyroAngle = 0;
  private double gyroOffset;
  private double frontAngle;

  private double startTime;
  private double distanceTraveled;

  private Boolean firstRun = true;

  private double leftFrontEncoderStart;
  private double rightFrontEncoderStart;
  private double leftBackEncoderStart;
  private double rightBackEncoderStart;

  private NetworkTableQuerier ntables;

  private Timer timer = new Timer();
  private PIDController pidFrontAngle;

  //Distance is in inches, target rotation is a value from -1 to 1 or and angle? 
  public AutoDrive(SwerveDrive drive, double speed, double dis, double ang, double rot, double heading, double time, NetworkTableQuerier table) {
   
    targetSpeed = speed;
    targetDriveDistance = dis;
    targetAngle = ang; //Divide by 360 to get a value from 0 to 1 for compatibility with SwerveDrive as a RightX
    chassisRotation = rot;
    frontAngle = heading / 360;
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

    //ntables.zeroPiGyro();

    gyroOffset = 0.0;

    //The constants for these need to be figured out
    pidFrontAngle = new PIDController(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (firstRun) {
      
      drivetrain.zeroEncoders();

      leftFrontEncoderStart = drivetrain.getLeftFrontDriveEncoder();
      rightFrontEncoderStart = drivetrain.getRightFrontDriveEncoder();
      leftBackEncoderStart = drivetrain.getLeftBackDriveEncoder();
      rightBackEncoderStart = drivetrain.getRightBackDriveEncoder();
  
      SmartDashboard.putNumber("Left Front Start", leftFrontEncoderStart);
      SmartDashboard.putNumber("Right Front Start", rightFrontEncoderStart);
      SmartDashboard.putNumber("Left Back Start", leftBackEncoderStart);
      SmartDashboard.putNumber("Right Back Start", rightBackEncoderStart);

      gyroOffset = 0;
      SmartDashboard.putNumber("Gyro Offset", gyroOffset);

      firstRun = false;
    }

    // Calculate angle correction based on gyro reading
    currentGyroAngle = 0  - gyroOffset;
    SmartDashboard.putNumber("Current Gyro", currentGyroAngle);
    
    targetRotation = pidFrontAngle.calculate(currentGyroAngle / 360.0, frontAngle) + chassisRotation;

    SmartDashboard.putNumber("Yaw", targetRotation);
    SmartDashboard.putNumber("Angle PID output", targetRotation);

    SmartDashboard.putNumber("DriveSpeed", targetSpeed);
    // Enforce minimum speed
    // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {
    //   angleCorrection = 0;
    //   if (driveSpeed < 0) {
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
    //double totalRotationsLeftFront = Math.abs((drivetrain.getLeftFrontDriveEncoder() - leftFrontEncoderStart)); //does getEncoderPosition return rotations or another unit?
    double totalRotationsLeftFront = drivetrain.getLeftFrontDriveEncoder(); //does getEncoderPosition return rotations or another unit?
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsLeftFront) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Left Front Rotations", distanceTraveled);

    //double totalRotationsRightFront = Math.abs((drivetrain.getRightFrontDriveEncoder() - rightFrontEncoderStart));
    double totalRotationsRightFront = (drivetrain.getRightFrontDriveEncoder());
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsRightFront) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Right Front Rotations", distanceTraveled);

    //double totalRotationsLeftBack = Math.abs((drivetrain.getLeftBackDriveEncoder() - leftBackEncoderStart));
    double totalRotationsLeftBack = (drivetrain.getLeftBackDriveEncoder());
    distanceTraveled = (kWheelDiameter * Math.PI * totalRotationsLeftBack) / (kTalonFXPPR * kDriveGearRatio);
    SmartDashboard.putNumber("Left Back Rotations", distanceTraveled);

    //double totalRotationsRightBack = Math.abs((drivetrain.getRightBackDriveEncoder() - rightBackEncoderStart));
    double totalRotationsRightBack = (drivetrain.getRightBackDriveEncoder());
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

