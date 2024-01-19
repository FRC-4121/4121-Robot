// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;


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

  private AHRS gyro;
  private MedianFilter gyro_filter;

  private double joystickDeadband;

  private double frontLeftAngle;
  private double backLeftAngle;
  private double frontRightAngle;
  private double backRightAngle;

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
    gyro = new AHRS(SPI.Port.kMXP);
    SmartDashboard.putNumber("Zero Gyro", 0);
    //gyro.calibrate();
    gyro.reset();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }

  //takes in the current controller input and calculates the new velocity and angle for all of the modules. 
  public void drive(double leftX, double leftY, double rightX) {

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
    double omegaRadiansPerSecond = (RotationalSpeed) * rightX;

    //Create Chassis Speed
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, Rotation2d.fromDegrees(getGyroAngle()));
  
    //ChassisSpeeds speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);


    SmartDashboard.putNumber("x meters per second",vxMetersPerSecond);
    SmartDashboard.putNumber("y meters per second",vyMetersPerSecond);
    SmartDashboard.putNumber("omega meters per second",omegaRadiansPerSecond);

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    /* 
    // Front left module state
    SwerveModuleState frontLeft = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(getGyroAngle()));

    // Front right module state
    SwerveModuleState frontRight = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(getGyroAngle()));

    // Back left module state
    SwerveModuleState backLeft = SwerveModuleState.optimize(moduleStates[2], new Rotation2d(getGyroAngle()));

    // Back right module state
    SwerveModuleState backRight = SwerveModuleState.optimize(moduleStates[3], new Rotation2d(getGyroAngle()));
    */

    frontLeftAngle = moduleStates[2].angle.getDegrees();
    frontRightAngle = moduleStates[3].angle.getDegrees();
    backLeftAngle =  moduleStates[0].angle.getDegrees();
    backRightAngle = moduleStates[1].angle.getDegrees();

    //Correcting negative angles to be within 0 to 360
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

        leftFront.drive(moduleStates[0].speedMetersPerSecond, frontLeftAngle);
        rightFront.drive(moduleStates[1].speedMetersPerSecond, frontRightAngle);
        leftBack.drive(moduleStates[2].speedMetersPerSecond, backLeftAngle);
        rightBack.drive(moduleStates[3].speedMetersPerSecond, backRightAngle);

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
      //gyro.calibrate();
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
