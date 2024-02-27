// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.MechanismConstants.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.*;


public class SwerveDriveWPI extends SubsystemBase {
  
  // Declare motor CAN IDs
  private final int kLeftFrontDrive = 1;
  private final int kLeftFrontAngle = 2;
  private final int kLeftFrontCoder = 3;
  private final int kRightFrontDrive = 4;
  private final int kRightFrontAngle = 5;
  private final int kRightFrontCoder = 6;
  private final int kRightBackDrive = 7;
  private final int kRightBackAngle = 8;
  private final int kRightBackCoder = 9;
  private final int kLeftBackDrive = 10;
  private final int kLeftBackAngle = 11;
  private final int kLeftBackCoder = 12;

  // Declare swerve modules
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;

  // Declare swerve kinematics objects
  private SwerveDriveKinematics kinematics;
  private Translation2d leftFrontTranslation;
  private Translation2d leftBackTranslation;
  private Translation2d rightFrontTranslation;
  private Translation2d rightBackTranslation;
  private ChassisSpeeds speeds;

  // Declare swerve module inputs
  private double frontLeftAngle;
  private double backLeftAngle;
  private double frontRightAngle;
  private double backRightAngle;

  // Declare NavX gyro Objects
  private AHRS gyro;
  private MedianFilter gyro_filter;
  private MedianFilter yaw_filter; 

  // Declare misc variables
  private double joystickDeadband;

  // Declare distance calculation variables
  private double LeftFrontStartingEncoder;
  private double LeftRearStartingEncoder;
  private double RightFrontStartingEncoder;
  private double RighRearStartingEncoder;

  // Declare collision detection variables;
  private double lastLinearAccelX;
  private double lastLinearAccelY;

  // Declare PID controller
  private PIDController wpiPIDController;

  /**
   *  
   * Creates a new SwerveDrive
   * 
  */
  public SwerveDriveWPI() {

    // Initialize new swerve modules
    leftFront = new SwerveWheel(kLeftFrontDrive, kLeftFrontAngle, kLeftFrontCoder);
    leftBack = new SwerveWheel(kLeftBackDrive, kLeftBackAngle, kLeftBackCoder);
    rightFront = new SwerveWheel(kRightFrontDrive, kRightFrontAngle, kRightFrontCoder);
    rightBack = new SwerveWheel(kRightBackDrive, kRightBackAngle, kRightBackCoder);

    // Initialize swerve kinematics objects
    leftFrontTranslation = new Translation2d(0.297,0.288);//X-0.297, Y-0.288 
    leftBackTranslation = new Translation2d(-0.297,0.288);//0.229,0.292 for last years bot
    rightFrontTranslation = new Translation2d(0.297,-0.288);
    rightBackTranslation = new Translation2d(-0.297,-0.288);
    kinematics = new SwerveDriveKinematics(leftFrontTranslation,rightFrontTranslation,leftBackTranslation,rightBackTranslation);

    // Initialize NavX gyro
    gyro = new AHRS(SPI.Port.kMXP);
    SmartDashboard.putNumber("Zero Gyro", 0);
    //gyro.calibrate();
    gyro.reset();
    gyro.resetDisplacement();
    //gyro.setAngleAdjustment(270);

    // Initialize gyro filter
    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
    yaw_filter = new MedianFilter(FILTER_WINDOW_SIZE);

    // Initialize misc variables
    joystickDeadband = 0.05;
    lastLinearAccelX = 0.0;
    lastLinearAccelY = 0.0;

    // Create PID controller
    wpiPIDController = new PIDController(kAnglePIDkp, kAnglePIDki, kAnglePIDkd);
    wpiPIDController.setTolerance(1.0, 5);

  }

  /**
   * 
   * Things to check/do every robot cycle
   * 
   */
  @Override
  public void periodic() {

    // Zero the gyro on driver command
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro == 1)
    {
      SmartDashboard.putNumber("Zero Gyro", 0);
      //gyro.calibrate();
      zeroGyro();
    }

  }

  /**
   * 
   * Moves the robot by calculating the speed and angle for each
   * swerve module based on joystick input
   * 
   * @param leftX  The left-right position of the left joystick. Controls slew.  This is Y direction (left +) in WPI coordinate system.
   * @param leftY  The up-down position of the left joystick. Controls forward/backward.  This is X direction (up +) in WPI coordinate system.
   * @param rightX  The left-right position of the right joystick. Controls rotation.
   * 
   */
  public void drive(double leftX, double leftY, double rightX) {

    // Put joystick positions on dashboard for testing/debugging
    SmartDashboard.putNumber("Left X", leftX);
    SmartDashboard.putNumber("Left Y", leftY);
    SmartDashboard.putNumber("Right X", rightX);

    //Convert speeds into their native units
    double vxMetersPerSecond = -(leftY * LinearSpeed);
    double vyMetersPerSecond = (leftX * LinearSpeed);
    System.out.println("Linear Speed: " + LinearSpeed);

    double vTotalMetersPerSecond = Math.sqrt((vxMetersPerSecond * vxMetersPerSecond) + (vyMetersPerSecond * vyMetersPerSecond));
    
    if(vTotalMetersPerSecond > LinearSpeed){
      vTotalMetersPerSecond = LinearSpeed;
    }

    //double omegaRadiansPerSecond = -((((-MaxRotationalSpeed / MaxLinearSpeed) * vTotalMetersPerSecond) + MaxRotationalSpeed) * rightX);
    double omegaRadiansPerSecond = 0.0;
    if(Math.abs(rightX) < kJoystickTolerance){
      double pidOutput = wpiPIDController.calculate(getGyroYawRate()*degreesToRads, 0.0); 
      omegaRadiansPerSecond = RotationalSpeed * pidOutput;
    } else{
      omegaRadiansPerSecond = RotationalSpeed * rightX;
    }
    
    //double omegaRadiansPerSecond = (RotationalSpeed) * (rightX + pidOutput);
    //double omegaRadiansPerSecond = (RotationalSpeed) * (rightX);

    // Put values on dashboard for testing/debugging
    SmartDashboard.putNumber("x meters per second",vxMetersPerSecond);
    SmartDashboard.putNumber("y meters per second",vyMetersPerSecond);
    SmartDashboard.putNumber("omega meters per second",omegaRadiansPerSecond);
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Yaw Rate", getGyroYawRate());
    SmartDashboard.putNumber("X Disp", (gyro.getDisplacementX()*100/2.54));//converted from meters to inches
    SmartDashboard.putNumber("Y Disp", (gyro.getDisplacementY()*100/2.54));//converted from meters to inches
  
    // Create Chassis Speed based on drive mode
    if (isFieldOriented) {

      //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroAngle()));
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroYaw()));

    }
    else {
    
      speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

    }
  
    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Get calculated module angles
    frontLeftAngle = moduleStates[2].angle.getDegrees();
    frontRightAngle = moduleStates[3].angle.getDegrees();
    backLeftAngle =  moduleStates[0].angle.getDegrees();
    backRightAngle = moduleStates[1].angle.getDegrees();

    // Correct negative angles to be within 0 to 360
    if (backRightAngle < 0)
    {
      backRightAngle = 360 + backRightAngle;
    }
    if (backLeftAngle < 0)
    {
      backLeftAngle = 360 + backLeftAngle;
    }
    if (frontRightAngle < 0)
    {
      frontRightAngle = 360 + frontRightAngle;
    }
    if (frontLeftAngle < 0)
    {
      frontLeftAngle = 360 + frontLeftAngle;
    }

    // Run the swerve modules based on current status 
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband && Math.abs(rightX) < joystickDeadband) {

      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();

    }
     else {

      if (!isParked) {

        leftFront.drive(moduleStates[0].speedMetersPerSecond, frontLeftAngle);
        rightFront.drive(moduleStates[1].speedMetersPerSecond, frontRightAngle);
        leftBack.drive(moduleStates[2].speedMetersPerSecond, backLeftAngle);
        rightBack.drive(moduleStates[3].speedMetersPerSecond, backRightAngle);

        // Put drive values on dashboard for testing/debugging
        SmartDashboard.putNumber("Left Front Speed", moduleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Left Front Angle", moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Corrected Left Front Angle", frontLeftAngle);
        SmartDashboard.putNumber("Right Front Speed", moduleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Right Front Angle", moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Corrected Right Front Angle", frontRightAngle);
        SmartDashboard.putNumber("Left Back Speed", moduleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Left Back Angle", moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Corrected Left Back Angle", backLeftAngle);
        SmartDashboard.putNumber("Right Back Speed", moduleStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("Right Back Angle", moduleStates[3].angle.getDegrees());
        SmartDashboard.putNumber("Corrected Right Back Angle", backRightAngle);

      }

    }

    // Check for collision
    double currLinearAccelX = gyro.getWorldLinearAccelX();
    double currentJerkX = currLinearAccelX - lastLinearAccelX;
    lastLinearAccelX = currLinearAccelX;
    double currLinearAccelY = gyro.getWorldLinearAccelY();
    double currentJerkY = currLinearAccelY - lastLinearAccelY;
    lastLinearAccelY = currLinearAccelY;
          
    if ( (Math.abs(currentJerkX) > kCollisionThresholdDeltaG ) || (Math.abs(currentJerkY) > kCollisionThresholdDeltaG) ) {

      impactDetected = true;

    }
  
  }

  /**
   * 
   * Stop all modules
   * 
   */
  public void stopDrive()
  {

    leftFront.stop();
    rightFront.stop();
    leftBack.stop();
    rightBack.stop();  

  }

  /**
   * 
   * Get the gyro angle and filter (smooth) it 
   * @return Smoothed gyro angle
   * 
   */
  public double getGyroAngle() {

    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    if(correctedGyro < 0)
    {
      correctedGyro = 360 + correctedGyro;
    }
    if (autoPosition == "Left") {
      SmartDashboard.putNumber("C-Gyro",correctedGyro + leftGyroCorrection);
      return (correctedGyro + leftGyroCorrection) % 360;
    }
    if (autoPosition == "Right") {
      SmartDashboard.putNumber("C-Gyro",correctedGyro + rightGyroCorrection);
      return (correctedGyro + rightGyroCorrection) % 360;
    }
    SmartDashboard.putNumber("C-Gyro",correctedGyro);
    return correctedGyro;

  }

  /**
   * Gets the gyro yaw angle (-180 to 180)
   * @return Corrected yaw angle
   */
  public double getGyroYaw() {

    // Get filtered yaw angle (in degrees)
    // Negate value to be consistent with WPI coordinate system
    double gyroYaw = -gyro_filter.calculate(gyro.getYaw());

    // Make sure we don't see -180
    if (gyroYaw == -180.0) {
      gyroYaw = 180.0;
    }

    // Correct angle for starting position
    if (autoPosition == "Left") {
      gyroYaw = gyroYaw + leftGyroCorrection;
    }
    if (autoPosition == "Right") {
      gyroYaw = gyroYaw + rightGyroCorrection;
    }

    // Put angle on the dashboard for driver
    SmartDashboard.putNumber("C-Gyro", gyroYaw);

    // Return yaw angle
    return gyroYaw;

  }

  /**
   * Get the current yaw rate of the gyro
   * @return
   */
  public double getGyroYawRate() {

    double yawRate = -yaw_filter.calculate(gyro.getRate());
    return yawRate;

  } 

  /**
   * 
   * Reset current gyro heading to zero
   * 
   */
  public void zeroGyro() {
    // gyro.calibrate();
    gyro.reset();

  }

  /*
   * Calibrate the gyro board
   */
  public void calibrateGyro() {
    gyro.calibrate();
  }

  /** 
   * 
   * Zero the encoders
   * 
   */
  public void zeroEncoders() {

    leftFront.zeroEncoder();
    rightFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightBack.zeroEncoder();

  }

  /**
   * 
   * Gets encoder value for left front drive motor
   * 
   */
  public double getLeftFrontDriveEncoder() {

    return leftFront.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for right front drive motor
   * 
   */
  public double getRightFrontDriveEncoder() {

    return rightFront.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for left back drive motor
   * 
   */
  public double getLeftBackDriveEncoder() {

    return leftBack.getDriveEncoderPosition();
  }

  /**
   * 
   * Gets encoder value for right back drive motor
   * 
   */
  public double getRightBackDriveEncoder() {

    return rightBack.getDriveEncoderPosition();
  }

  /**
   * 
   * Park the bot so it doesn't move
   * 
   */
  public void parkBot()
  {

    leftFront.drive(0, 135);
    rightFront.drive(0, 225);
    leftBack.drive(0, 45);
    rightBack.drive(0, 315);
  }

  /*
   * Set the starting values for a drive distance calculation
   */
  public void resetDistance() {

    LeftFrontStartingEncoder = getLeftFrontDriveEncoder();
    LeftRearStartingEncoder = getLeftBackDriveEncoder();
    RightFrontStartingEncoder = getRightFrontDriveEncoder();
    RighRearStartingEncoder = getRightBackDriveEncoder();

  }

  /*
   * Calculate the drive distance
   */
  public double calculateDriveDistance() {

    // Initialize return value
    double driveDistance = 0.0;
    
    // Get total rotations driven since last distance reset
    double totalRotationsLeftFront = getLeftFrontDriveEncoder() - LeftFrontStartingEncoder;
    double totalRotationsRightFront = getRightFrontDriveEncoder() - RightFrontStartingEncoder;
    double totalRotationsLeftBack = getLeftBackDriveEncoder() - LeftRearStartingEncoder;
    double totalRotationsRightBack = getRightBackDriveEncoder() - RighRearStartingEncoder;

    // Calculate distance by averaging encoders
    double averageRotations = ((totalRotationsLeftFront + totalRotationsRightFront + totalRotationsLeftBack + totalRotationsRightBack) / 4.0);
    driveDistance = (kWheelDiameter * Math.PI * averageRotations) / (kTalonFXPPR * kDriveGearRatio);

    // Return calculated distance
    return driveDistance;

  }
  
}
