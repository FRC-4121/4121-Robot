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
  private double maxYawRate = 7.0;
  private double yawDrift;

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

    // Initialize gyro filter
    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
    yaw_filter = new MedianFilter(FILTER_WINDOW_SIZE);

    // Initialize misc variables
    joystickDeadband = 0.05;

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
   * @param leftX  The left-right position of the left joystick. Controls slew.
   * @param leftY  The up-down position of the left joystick. Controls forward/backward.
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

    double vTotalMetersPerSecond = Math.sqrt((vxMetersPerSecond * vxMetersPerSecond) + (vyMetersPerSecond * vyMetersPerSecond));
    
    if(vTotalMetersPerSecond > LinearSpeed){
      vTotalMetersPerSecond = LinearSpeed;
    }

    //double omegaRadiansPerSecond = -((((-MaxRotationalSpeed / MaxLinearSpeed) * vTotalMetersPerSecond) + MaxRotationalSpeed) * rightX);
    if(rightX == 0){
      yawDrift = getGyroYawRate();
    } else{
      yawDrift = 0;
    }
    
    double omegaRadiansPerSecond = (RotationalSpeed) * (rightX + (yawDrift/maxYawRate));
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

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroAngle()));

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
    if (autoPosition == "Left") {
      SmartDashboard.putNumber("C-Gyro",correctedGyro + leftGyroCorrection);
      return correctedGyro + leftGyroCorrection;
    }
    if (autoPosition == "Right") {
      SmartDashboard.putNumber("C-Gyro",correctedGyro + rightGyroCorrection);
      return correctedGyro + rightGyroCorrection;
    }
    SmartDashboard.putNumber("C-Gyro",correctedGyro);
    return correctedGyro;

  }

  /**
   * Get the current yaw rate of the gyro
   * @return
   */
  public double getGyroYawRate() {

    double yawRate = yaw_filter.calculate(gyro.getRate());
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
  
}
