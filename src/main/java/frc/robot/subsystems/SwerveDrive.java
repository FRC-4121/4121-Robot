// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.filter.MedianFilter;

/**
 * @deprecated SwerveDriveWPI should be used instead
 */
@Deprecated
public class SwerveDrive extends SubsystemBase {
  private SwerveWheel leftFront;
  private SwerveWheel leftBack;
  private SwerveWheel rightFront;
  private SwerveWheel rightBack;

  private ADXRS450_Gyro gyro;
  private MedianFilter gyro_filter;

  private double joystickDeadband;
  
  private static final int leftFrontDriveId = 1;
  private static final int leftFrontAngleId = 2;
  private static final int leftFrontCoderId = 3;
  
  private static final int rightFrontDriveId = 4;
  private static final int rightFrontAngleId = 5;
  private static final int rightFrontCoderId = 6;

  private static final int rightBackDriveId = 7;
  private static final int rightBackAngleId = 8;
  private static final int rightBackCoderId = 9;

  private static final int leftBackDriveId = 10;
  private static final int leftBackAngleId = 11;
  private static final int leftBackCoderId = 12;

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    leftFront = new SwerveWheel(
      SwerveWheel.baseConfig.clone()
        .withName("LF")
        .withIds(leftFrontDriveId, leftFrontAngleId, leftFrontCoderId)
    );
    leftBack = new SwerveWheel(
      SwerveWheel.baseConfig.clone()
        .withName("LB")
        .withIds(leftBackDriveId, leftBackAngleId, leftBackCoderId)
    );
    rightFront = new SwerveWheel(
      SwerveWheel.baseConfig.clone()
        .withName("RF")
        .withIds(rightFrontDriveId, rightFrontAngleId, rightFrontCoderId)
    );
    rightBack = new SwerveWheel(
      SwerveWheel.baseConfig.clone()
        .withName("RB")
        .withIds(rightBackDriveId, rightBackAngleId, rightBackCoderId)
    );

    joystickDeadband = 0.05;

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    gyro.reset();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }

  /**
   * Take joystick inputs, like those from a controller, and use them to drive.
   * @param leftX the X-value on the left stick, controls left-right motion
   * @param leftY the Y-value on the left stick, controls forward-backward motion
   * @param rightX the X-value on the right stick, controls rotation
   */
  public void driveFromContoller(double leftX, double leftY, double rightX) {
    SmartDashboard.putNumber("Left X", leftX);
    SmartDashboard.putNumber("Left Y", leftY);
    SmartDashboard.putNumber("Right X", rightX);

    double rate = Math.sqrt((lengthFromAxle * lengthFromAxle) + (widthFromAxle * widthFromAxle));
    leftY *= -1;

    double a = leftX - rightX * (lengthFromAxle / rate);
    double b = leftX + rightX * (lengthFromAxle / rate);
    double c = leftY - rightX * (widthFromAxle / rate);
    double d = leftY + rightX * (widthFromAxle / rate);    

    // Calculate wheel speeds from 0 to 1
    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));    

    // Cap wheel speeds at 1
    if (backRightSpeed > 1.0)
      backRightSpeed = 1.0;
    if (backLeftSpeed > 1.0)
      backLeftSpeed = 1.0;
    if (frontRightSpeed > 1.0)
      frontRightSpeed = 1.0;
    if (frontLeftSpeed > 1.0)
      frontLeftSpeed = 1.0;

    // Calculate Wheel Angles -180 to 180
    double backLeftAngle = Math.atan2 (a, d) / Math.PI * 180;
    double backRightAngle = Math.atan2 (a, c) / Math.PI * 180;
    double frontLeftAngle = Math.atan2 (b, d) / Math.PI * 180;
    double frontRightAngle = Math.atan2 (b, c) / Math.PI * 180;

    // Correct negative angles to be within 0 to 360
    if (backRightAngle < 0)
      backRightAngle += 360;
    if (backLeftAngle < 0)
      backLeftAngle += 360;
    if (frontRightAngle < 0)
      frontRightAngle += 360;
    if (frontLeftAngle < 0)
      frontLeftAngle += 360;

    SmartDashboard.putNumber("back right speed", backRightSpeed);
    SmartDashboard.putNumber("front right speed", frontRightSpeed);
    SmartDashboard.putNumber("back left speed", backLeftSpeed);
    SmartDashboard.putNumber("front left speed", frontLeftSpeed);

    SmartDashboard.putNumber("back right angle", backRightAngle);
    SmartDashboard.putNumber("front right angle", frontRightAngle);
    SmartDashboard.putNumber("back left angle", backLeftAngle);
    SmartDashboard.putNumber("front left angle", frontLeftAngle);

    //Checking if robot is in park mode 
    if (Math.abs(leftX) < joystickDeadband && Math.abs(leftY) < joystickDeadband && Math.abs(rightX) < joystickDeadband) {
      // Stop motors without turning wheels
      stopDrive();
    } else {
      if (!isParked) {
        leftFront.drive(frontLeftSpeed, frontLeftAngle);
        rightFront.drive(frontRightSpeed, frontRightAngle);
        leftBack.drive(backLeftSpeed, backLeftAngle);
        rightBack.drive(backRightSpeed, backRightAngle);
      }
    }

    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  }

  /**
   * This is supposed to take values from AutoDrive and drive with them, but doesn't do anything right now
   * @deprecated This function doesn't do anything, and will be removed later in my cleanup
   * @param leftX
   * @param leftY
   * @param rightX
  */
  @Deprecated
  public void autoDrive(double leftX, double leftY, double rightX) {
    // probably concerning that there's nothing here
  }

  public void stopDrive() {
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
    // We use a number field rather than boolean because the driver station doesn't allow boolean input
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro != 0) {
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

  //Method to park the bot so it doesn't move
  public void parkBot() {
    leftFront.drive(0, 135);
    rightFront.drive(0, 225);
    leftBack.drive(0, 45);
    rightBack.drive(0, 315);
  }

}
