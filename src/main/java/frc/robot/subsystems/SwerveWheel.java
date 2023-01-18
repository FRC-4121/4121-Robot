// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.ExtraClasses.*;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoder.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWheel extends SubsystemBase { 

  /* Declare motor variables */
  private WPI_TalonFX swerveDriveMotor;
  private WPI_TalonFX swerveAngleMotor;
  private CANCoder canCoder;

  /* Declare controller variables */
  private PIDControl drivePIDController;
  private PIDControl anglePIDController;

  /* Declare state variables */
  private double wheelSpeed;
  private double wheelAngle;
  private double wheelID;

  /* Creates a new SwerveWheel. */
  public SwerveWheel(int driveMotorID, int angleMotorID, int CANCoderID) {
    wheelID = CANCoderID / 3;
    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID, CANCoderID);

    // Initialize PID controllers
    drivePIDController = new PIDControl(kP_SwerveDriveSpeed, kI_SwerveDriveSpeed, kD_SwerveDriveSpeed);
    anglePIDController = new PIDControl(kP_SwerveDriveAngle, kI_SwerveDriveAngle, kD_SwerveDriveAngle);

  }

  /* Initialize motors */
  private void InitSwerveMotors(int driveMotorID, int angleMotorID, int CANCoderID) {

    // Create motors
    swerveDriveMotor = new WPI_TalonFX(driveMotorID);
    swerveAngleMotor = new WPI_TalonFX(angleMotorID);
    canCoder = new CANCoder(CANCoderID);

    // Set brake mode
    swerveDriveMotor.setNeutralMode(NeutralMode.Brake);
    swerveAngleMotor.setNeutralMode(NeutralMode.Brake);

    // Set motor encoders
    swerveDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    swerveAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
  
  }
 
  /* Periodically set wheel speed and angle */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void drive(double speed, double angle){
    
    //Running the swerve drive motors with a limit for speed
    swerveDriveMotor.set(speed * swerveDriveSpeedLimiter);

    double encoderAngle = canCoder.getAbsolutePosition();
    
    //Putting CANCoder position on smart dashboard
    SmartDashboard.putNumber("Wheel "+ wheelID, encoderAngle);

    double target = angle;

    // adjust to always take the shortest path
    if (target - encoderAngle > 180) target -= 360;
    if (encoderAngle - target > 180) target += 360;
    
    
    double output = anglePIDController.run(encoderAngle,target);
    SmartDashboard.putNumber("Wheel " + wheelID + " angle output", output);
    
    double angleSpeed = output * swerveDriveAngleLimiter;
    
    //Capping angleSpeed to max that the motor can take, from -1 to 1
    if(angleSpeed > 1)
    {
      angleSpeed = 1;
    } else if(angleSpeed < -1)
    {
      angleSpeed = -1;
    }
    
    
    swerveAngleMotor.set(angleSpeed);
    
  }

  public void drive2(double speed, double angle){
    
    //Running the swerve drive motors with a limit for speed
    swerveDriveMotor.set(speed * swerveDriveSpeedLimiter);
    wheelAngle = angle;

    double Angle = canCoder.getAbsolutePosition();
    double target = wheelAngle;

    // adjust to always take the shortest path
    if (target - Angle > 180) target -= 360;
    if (Angle - target > 180) target += 360;

    double output = anglePIDController.run(Angle, wheelAngle);
    // double output = target;
    //double output = (target - Angle) / 360 * kP_SwerveDriveAngle;
    swerveAngleMotor.set(output / 3);
  }
  public void driveRect(double x, double y, double a) {
    drive(Math.sqrt(x * x + y * y), Math.atan2(y, x) / Math.PI * 180 + a);
  }
}
