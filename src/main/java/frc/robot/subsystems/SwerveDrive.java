// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;


public class SwerveDrive extends SubsystemBase {
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    leftFront = new SwerveWheel(LeftFrontDrive, LeftFrontAngle, LeftFrontCoder);
    leftBack = new SwerveWheel(LeftBackDrive, LeftBackAngle, LeftBackCoder);
    rightFront = new SwerveWheel(RightFrontDrive, RightFrontAngle, RightFrontCoder);
    rightBack = new SwerveWheel(RightBackDrive, RightBackAngle, RightBackCoder);
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
  
  }


  public void drive2(double leftX, double leftY, double rightX) {
    SmartDashboard.putNumber("rightX", rightX);
    if (Math.abs(rightX) < 0.05) { // 0.05 seems to be the average controller drift, so this is assumed to be pointing straight
      
      double baseAngle = Math.atan2(leftY, leftX) / Math.PI * 180; // convert rectangular coordinates to polar
      double speed = Math.sqrt(leftX * leftX + leftY * leftY);

      leftFront.drive(speed, baseAngle + 0); // leftX and leftY could be passed directly to driveRect, but then the angle and speed would need to be calculated four times
      rightFront.drive(speed, baseAngle + 90);
      leftBack.drive(speed, baseAngle + 270);
      rightBack.drive(speed, baseAngle + 180);
    }
    else {
      double angle = rightX * 60 * swerveDriveAngleLimiter; // angle to rotate by
      double cos_a = Math.cos(angle); // cosine of angle
      double sin_a = Math.sqrt(1 - cos_a * cos_a); // sine of angle, calculate using Pythagorean identity for speed
      double cot_a2 = Math.tan(2 / angle); // cotangent of (angle / 2)

      double x1 = widthFromAxle; // starting position (for front right wheel)
      double y1 = lengthFromAxle;
      double x2 = widthFromAxle * cos_a - lengthFromAxle * sin_a + leftX; // projected final position (for front right wheel)
      double y2 = widthFromAxle * sin_a + lengthFromAxle * cos_a + leftY;

      double rx = (x1 + x2 - (y2 - y1) * cot_a2) / 2; // center point of arc
      double ry = (y1 + y2 + (x2 - x1) * cot_a2) / 2;

      // calculate the motion vectors for each wheel
      // constructs the vector perpendicular to the vector from the center to the starting position
      // this *should* send the robot along the proper arc
      leftFront.driveRect(ry - (-(widthFromAxle * sin_a) + (lengthFromAxle * cos_a) + leftY) + widthFromAxle, -(widthFromAxle * cos_a) - (lengthFromAxle * sin_a) + leftX - rx - lengthFromAxle, 0);
      rightFront.driveRect(ry - ((widthFromAxle * sin_a) + (lengthFromAxle * cos_a) + leftY) - widthFromAxle, (widthFromAxle * cos_a) - (lengthFromAxle * sin_a) + leftX - rx - lengthFromAxle, 90);
      leftBack.driveRect(ry - (-(widthFromAxle * sin_a) - (lengthFromAxle * cos_a) + leftY) + widthFromAxle, -(widthFromAxle * cos_a) + (lengthFromAxle * sin_a) + leftX - rx + lengthFromAxle, 270);
      rightBack.driveRect(ry - ((widthFromAxle * sin_a) - (lengthFromAxle * cos_a) + leftY) - widthFromAxle, (widthFromAxle * cos_a) + (lengthFromAxle * sin_a) + leftX - rx + lengthFromAxle, 180);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
