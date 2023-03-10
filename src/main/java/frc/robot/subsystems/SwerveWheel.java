// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.*;
import static frc.robot.Constants.*;
import frc.robot.ExtraClasses.*;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoder.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.*;

public class SwerveWheel extends SubsystemBase { 

  /* Declare motor variables */
  private WPI_TalonFX swerveDriveMotor;
  private WPI_TalonFX swerveAngleMotor;
  private CANCoder canCoder;

  /* Declare controller variables */
  private PIDControl anglePIDController;
  private PIDController wpiPIDController;
  
  private double angleSpeedLimiter;
  private double kP_AngleController;
  private double kI_AngleController;
  private double kD_AngleController;


  /* Declare state variables */
  private double wheelSpeed;
  private double wheelAngle;
  private int wheelID;

  private MedianFilter angle_filter;
 

  /* Creates a new SwerveWheel. */
  public SwerveWheel(int driveMotorID, int angleMotorID, int CANCoderID) {
    
    // Set wheel ID
    wheelID = CANCoderID / 3;

    // Set controller constants
    angleSpeedLimiter = angleLimiters[wheelID - 1];
    kP_AngleController = anglePIDkPs[wheelID - 1];
    kI_AngleController = anglePIDkIs[wheelID - 1];
    kD_AngleController = anglePIDkDs[wheelID - 1];
    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID, CANCoderID);

    // Initialize PID controller
    anglePIDController = new PIDControl(kP_AngleController, kI_AngleController, kD_AngleController);
    wpiPIDController = new PIDController(kP_AngleController, kI_AngleController, kD_AngleController);
    wpiPIDController.setTolerance(1.5,5);

    SmartDashboard.putNumber("Wheel "+ wheelID + " kP", kP_AngleController);
    SmartDashboard.putNumber("Wheel "+ wheelID + " kI", kI_AngleController);
    SmartDashboard.putNumber("Wheel "+ wheelID + " kD", kD_AngleController);

    angle_filter = new MedianFilter(FILTER_WINDOW_SIZE);

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

    //Invert angle motor so that positive input moves the wheel clockwise
    swerveAngleMotor.setInverted(kMotorInvert);

    // Set motor encoders
    swerveDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    swerveAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    // swerveDriveMotor.config_kP(0, drivePIDkPs[wheelID - 1], kTimeoutMsDrive);
    // swerveDriveMotor.config_kI(0, drivePIDkIs[wheelID - 1], kTimeoutMsDrive);
    // swerveDriveMotor.config_kD(0, drivePIDkDs[wheelID - 1], kTimeoutMsDrive);
    swerveDriveMotor.config_kF(0, drivePIDkFs[wheelID - 1], kTimeoutMsDrive);

  }
 
  /* Periodically set wheel speed and angle */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void drive(double speed, double angle) {

    //Normalize target to have a max value of 1
    double target = angle / 360.0;
    if (target == 1.0) {
      target = 0.0;
    }

    //Normalize encoder to have a max value of 1 and correct for discontinuity at 360 degrees (should be 0)
    double encoderAngle = canCoder.getAbsolutePosition() / 360;
    if (encoderAngle == 1.0) {
      encoderAngle = 0.0;
    }

    //Putting CANCoder position on smart dashboard
    SmartDashboard.putNumber("Wheel "+ wheelID, encoderAngle);
  
    //Determine shortest rotation distance
    if (Math.abs(target - encoderAngle) > 0.5)
    {
      double diff = 1 - Math.abs(target - encoderAngle);
      target = target + diff;
      if (target >= 1.0) {
        target = target - 1.0;
      }
      encoderAngle = encoderAngle + diff;
      if (encoderAngle >= 1.0)
      {
        encoderAngle = encoderAngle - 1.0;
      }
    }

    //double output = anglePIDController.run(encoderAngle,target);
    double output = wpiPIDController.calculate(encoderAngle,target);
    SmartDashboard.putNumber("Wheel " + wheelID + " PID output", output);
    SmartDashboard.putNumber("Wheel " + wheelID + " Target", target);
    SmartDashboard.putNumber("Wheel " + wheelID + " Corrected Encoder", encoderAngle);
    
    double angleSpeed = output * angleSpeedLimiter;
    
    //Capping angleSpeed to max that the motor can take, from -1 to 1
    if (angleSpeed > 1)
    {
      angleSpeed = 1;
    } else if (angleSpeed < -1)
    {
      angleSpeed = -1;
    }

    //Before angle speed
    SmartDashboard.putNumber("Wheel " + wheelID + " before angle", angleSpeed);

    //Get the sign of the angleSpeed
    double sign = Math.signum(angleSpeed);

    //Enforce minimum angle speed, 0.0028 is one degree
    // double angleDiff = target - encoderAngle;
    // double angleError = Math.abs(angleDiff);
    // if (angleError < 0.01)
    // {
    //   if (angleError > 0.001)
    //   {
    //     angleSpeed = 0.06 * sign;
    //   }
    //   else
    //   {
    //     angleSpeed = 0.0;
    //   }
    // } 
    // else 
    // {
    //   angleSpeed = 0;
    // }

    //putting angleSpeed and error into smart dashboard
    SmartDashboard.putNumber("Wheel " + wheelID + " after angle", angleSpeed);
    SmartDashboard.putNumber("Wheel " + wheelID + " error", encoderAngle - target);
    
    //Calculate wheel velocity
    double wheelVelocity = 23712 * (speed * swerveDriveSpeedLimiter) - 894.29;
    SmartDashboard.putNumber("Wheel " + wheelID + " V target", wheelVelocity);

    //Set motor speeds
    swerveAngleMotor.set(angleSpeed);
    swerveDriveMotor.set(ControlMode.Velocity, wheelVelocity);
    SmartDashboard.putNumber("Wheel " + wheelID + " V actual", swerveDriveMotor.getSelectedSensorVelocity());
    
  }


  //Get the encoder value of the drive motor
  public double getDriveEncoderPosition() {

    return swerveDriveMotor.getSelectedSensorPosition();

  }

  //Zeros the encoder for the drive motor
  public void zeroEncoder() {

    swerveDriveMotor.setSelectedSensorPosition(0);
  }

  public void stop() {

    swerveDriveMotor.set(0);
    swerveAngleMotor.set(0);
  }

  
}
