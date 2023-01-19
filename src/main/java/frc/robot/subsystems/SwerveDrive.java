// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.filter.MedianFilter;

public class SwerveDrive extends SubsystemBase {
  
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;

  private ADXRS450_Gyro gyro;
  private MedianFilter gyro_filter;
  

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

    leftFront = new SwerveWheel(LeftFrontDrive, LeftFrontAngle, LeftFrontCoder);
    leftBack = new SwerveWheel(LeftBackDrive, LeftBackAngle, LeftBackCoder);
    rightFront = new SwerveWheel(RightFrontDrive, RightFrontAngle, RightFrontCoder);
    rightBack = new SwerveWheel(RightBackDrive, RightBackAngle, RightBackCoder);

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    gyro.reset();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }

  //takes in the current controller input and calculates the new velocity and angle for all of the modules. 
  public void drive(double leftX, double leftY, double rightX) {

    double rate = Math.sqrt((lengthFromAxle * lengthFromAxle) + (widthFromAxle * widthFromAxle));
    leftY *= -1;

    double a = leftX - rightX * (lengthFromAxle / rate);
    double b = leftX + rightX * (lengthFromAxle / rate);
    double c = leftY - rightX * (widthFromAxle / rate);
    double d = leftY + rightX * (widthFromAxle / rate);    

    //Calculate wheel speeds from 0 to 1
    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));    

    //Cap wheel speeds at 1
    if(backRightSpeed > 1.0){
      backRightSpeed = 1.0;
    } 
    if(backLeftSpeed > 1.0){
      backLeftSpeed = 1.0;
    } 
    if(frontRightSpeed > 1.0){
      frontRightSpeed = 1.0;
    } 
    if(frontLeftSpeed > 1.0){
      frontLeftSpeed = 1.0;
    } 


    //Calculate Wheel Angles -180 to 180
    double backLeftAngle = Math.atan2 (a, d) / Math.PI * 180;
    double backRightAngle = Math.atan2 (a, c) / Math.PI * 180;
    double frontLeftAngle = Math.atan2 (b, d) / Math.PI * 180;
    double frontRightAngle = Math.atan2 (b, c) / Math.PI * 180;

    //Correcting negative angles to be within 0 to 360
    if(backRightAngle < 0)
    {
      backRightAngle = 360 + backRightAngle;
    }
    if(backLeftAngle < 0)
    {
      backLeftAngle = 360 + backLeftAngle;
    }
    if(frontRightAngle < 0)
    {
      frontRightAngle = 360 + frontRightAngle;
    }
    if(frontLeftAngle < 0)
    {
      frontLeftAngle = 360 + frontLeftAngle;
    }

    SmartDashboard.putNumber("back right speed", backRightSpeed);
    SmartDashboard.putNumber("front right speed", frontRightSpeed);
    SmartDashboard.putNumber("back left speed", backLeftSpeed);
    SmartDashboard.putNumber("front left speed", frontLeftSpeed);

    SmartDashboard.putNumber("back right angle", backRightAngle);
    SmartDashboard.putNumber("front right angle", frontRightAngle);
    SmartDashboard.putNumber("back left angle", backLeftAngle);
    SmartDashboard.putNumber("front left angle", frontLeftAngle);

    leftFront.drive(frontLeftSpeed, frontLeftAngle);
    rightFront.drive(frontRightSpeed, frontRightAngle);
    leftBack.drive(backLeftSpeed, backLeftAngle);
    rightBack.drive(backRightSpeed, backRightAngle);

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  
  }


  public double getGyroAngle(){

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
  public void zeroGyro(){
    // gyro.calibrate();
    gyro.reset();

  }
}
