// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;


public class SwerveDriveWPI extends SubsystemBase {
  
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;

  private Translation2d leftFrontTranslation;
  private Translation2d leftBackTranslation;
  private Translation2d rightFrontTranslation;
  private Translation2d rightBackTranslation;

  private SwerveDriveKinematics kinematics;

  private ADXRS450_Gyro gyro;
  private MedianFilter gyro_filter;

  private double joystickDeadband;

  
  

  /** Creates a new SwerveDrive. */
  public SwerveDriveWPI() {

    leftFront = new SwerveWheel(LeftFrontDrive, LeftFrontAngle, LeftFrontCoder);
    leftBack = new SwerveWheel(LeftBackDrive, LeftBackAngle, LeftBackCoder);
    rightFront = new SwerveWheel(RightFrontDrive, RightFrontAngle, RightFrontCoder);
    rightBack = new SwerveWheel(RightBackDrive, RightBackAngle, RightBackCoder);

    leftFrontTranslation = new Translation2d(0.292,0.229);
    leftBackTranslation = new Translation2d(-0.292,0.229);
    rightFrontTranslation = new Translation2d(0.292,-0.229);
    rightBackTranslation = new Translation2d(-0.292,-0.229);

    kinematics = new SwerveDriveKinematics(leftFrontTranslation,rightFrontTranslation,leftBackTranslation,rightBackTranslation);

    joystickDeadband = 0.05;

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    gyro.reset();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }

  //takes in the current controller input and calculates the new velocity and angle for all of the modules. 
  public void drive(double leftX, double leftY, double rightX) {

    SmartDashboard.putNumber("Left X", leftX);
    SmartDashboard.putNumber("Left Y", leftY);
    SmartDashboard.putNumber("Right X", rightX);


    //Convert speeds into their native units
    double vxMetersPerSecond = -(leftX * MaxLinearSpeed);
    double vyMetersPerSecond = -(leftY * MaxLinearSpeed);

    double vTotalMetersPerSecond = Math.sqrt((vxMetersPerSecond * vxMetersPerSecond) + (vyMetersPerSecond * vyMetersPerSecond));
    
    if(vTotalMetersPerSecond > MaxLinearSpeed){
      vTotalMetersPerSecond = MaxLinearSpeed;
    }

    double omegaRadiansPerSecond = -((((-MaxRotationalSpeed / MaxLinearSpeed) * vTotalMetersPerSecond) + MaxRotationalSpeed) * rightX);
   

    //Create Chassis Speed
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroAngle()));

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(getGyroAngle()));

    // Front right module state
    SwerveModuleState frontRight = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(getGyroAngle()));

    // Back left module state
    SwerveModuleState backLeft = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(getGyroAngle()));

    // Back right module state
    SwerveModuleState backRight = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(getGyroAngle()));

    //Checking if robot is in park mode 
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband && Math.abs(rightX) < joystickDeadband) {

      // Stop motors without turning wheels
      leftFront.stop();
      rightFront.stop();
      leftBack.stop();
      rightBack.stop();

    }
     else {

      if (!isParked) {
        leftFront.drive(frontLeft.speedMetersPerSecond, frontLeft.angle.getDegrees());
        rightFront.drive(frontRight.speedMetersPerSecond, frontRight.angle.getDegrees());
        leftBack.drive(backLeft.speedMetersPerSecond, backLeft.angle.getDegrees());
        rightBack.drive(backRight.speedMetersPerSecond, backRight.angle.getDegrees());
      }

    }

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  
  }


  public void stopDrive()
  {

    leftFront.stop();
    rightFront.stop();
    leftBack.stop();
    rightBack.stop();  

  }

  public double getGyroAngle() {

    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    return correctedGyro;

  }

 
  @Override
  public void periodic() {

    // Zero the gyro on driver command
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro == 1)
    {
      SmartDashboard.putNumber("Zero Gyro", 0);
      gyro.calibrate();
      zeroGyro();
    }

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

  /** Zero the encoders */
  public void zeroEncoders() {

    leftFront.zeroEncoder();
    rightFront.zeroEncoder();
    leftBack.zeroEncoder();
    rightBack.zeroEncoder();

  }

  //Gets encoder value for left front drive motor
  public double getLeftFrontDriveEncoder() {

    return leftFront.getDriveEncoderPosition();
  }

  //Gets encoder value for right front drive motor
  public double getRightFrontDriveEncoder() {

    return rightFront.getDriveEncoderPosition();
  }

  //Gets encoder value for left back drive motor
  public double getLeftBackDriveEncoder() {

    return leftBack.getDriveEncoderPosition();
  }

  //Gets encoder value for right back drive motor
  public double getRightBackDriveEncoder() {

    return rightBack.getDriveEncoderPosition();
  }

  //Method to park the bot so it doesn't move
  public void parkBot()
  {

    leftFront.drive(0, 135);
    rightFront.drive(0, 225);
    leftBack.drive(0, 45);
    rightBack.drive(0, 315);
  }
}
