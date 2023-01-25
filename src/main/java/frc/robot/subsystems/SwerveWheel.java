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
  
  private double angleSpeedLimiter;
  private double kP_AngleController;
  private double kI_AngleController;
  private double kD_AngleController;


  /* Declare state variables */
  private double wheelSpeed;
  private double wheelAngle;
  private double wheelID;

  /* Creates a new SwerveWheel. */
  public SwerveWheel(int driveMotorID, int angleMotorID, int CANCoderID) {
    
    // Set wheel ID
    wheelID = CANCoderID / 3;

    // Set controller constants
    if(wheelID == 1)
    {
      angleSpeedLimiter = lfSwerveDriveAngleLimiter;
      kP_AngleController = kP_lfSwerveDriveAngle;
      kI_AngleController = kI_lfSwerveDriveAngle;
      kD_AngleController = kD_lfSwerveDriveAngle;
    } else if(wheelID == 2)
    {
      angleSpeedLimiter = rfSwerveDriveAngleLimiter;
      kP_AngleController = kP_rfSwerveDriveAngle;
      kI_AngleController = kI_rfSwerveDriveAngle;
      kD_AngleController = kD_rfSwerveDriveAngle;
    } else if(wheelID == 3)
    {
      angleSpeedLimiter = rbSwerveDriveAngleLimiter;
      kP_AngleController = kP_rbSwerveDriveAngle;
      kI_AngleController = kI_rbSwerveDriveAngle;
      kD_AngleController = kD_rbSwerveDriveAngle;
    } else if(wheelID == 4)
    {
      angleSpeedLimiter = lbSwerveDriveAngleLimiter;
      kP_AngleController = kP_lbSwerveDriveAngle;
      kI_AngleController = kI_lbSwerveDriveAngle;
      kD_AngleController = kD_lbSwerveDriveAngle;
    }

    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID, CANCoderID);

    // Initialize PID controllers
    drivePIDController = new PIDControl(kP_SwerveDriveSpeed, kI_SwerveDriveSpeed, kD_SwerveDriveSpeed);
    anglePIDController = new PIDControl(kP_AngleController, kI_AngleController, kD_AngleController);

    SmartDashboard.putNumber("Wheel "+ wheelID + " kP", kP_AngleController);
    SmartDashboard.putNumber("Wheel "+ wheelID + " kI", kI_AngleController);
    SmartDashboard.putNumber("Wheel "+ wheelID + " kD", kD_AngleController);

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

    //Normalize target to have a max value of 1
    double target = angle / 360.0;

    //Normalize encoder to have a max value of 1 and correct for discontinuity at 360 degrees (should be 0)
    double encoderAngle = canCoder.getAbsolutePosition() / 360;
    if(encoderAngle == 1.0){
      encoderAngle = 0.0;
    }

    //Putting CANCoder position on smart dashboard
    SmartDashboard.putNumber("Wheel "+ wheelID, encoderAngle);
  
    //Determine shortest rotation distance
    if(Math.abs(target - encoderAngle) > 0.5)
    {
      double diff = 1 - Math.abs(target - encoderAngle);
      target = target + diff;
      encoderAngle = encoderAngle + diff;
      if (encoderAngle == 1.0)
      {
        encoderAngle = 0.0;
      }
    }

    double output = anglePIDController.run(encoderAngle,target);
    SmartDashboard.putNumber("Wheel " + wheelID + " PID output", output);
    SmartDashboard.putNumber("Wheel " + wheelID + " Target", target);
    SmartDashboard.putNumber("Wheel " + wheelID + " Corrected Encoder", encoderAngle);
    
    double angleSpeed = output * angleSpeedLimiter;
    
    //Capping angleSpeed to max that the motor can take, from -1 to 1
    // if(angleSpeed > 1)
    // {
    //   angleSpeed = 1;
    // } else if(angleSpeed < -1)
    // {
    //   angleSpeed = -1;
    // }

    //Before angle speed
    SmartDashboard.putNumber("Wheel " + wheelID + " before angle", angleSpeed);

    //Get the sign of the angleSpeed
    // double sign = Math.signum(angleSpeed);

    //Enforce minimum angle speed, 0.0028 is one degree
    if(Math.abs(target - encoderAngle) > 0.009)
    {
      // if(angleSpeed < 0.06)
      // {
      //   angleSpeed = 0.06 * sign;
      // }
    } 
    else 
    {
      angleSpeed = 0;
    }

    //putting angleSpeed and error into smart dashboard
    SmartDashboard.putNumber("Wheel " + wheelID + " after angle", angleSpeed);
    SmartDashboard.putNumber("Wheel " + wheelID + " error", encoderAngle - target);
    
    //Set motor speeds
    swerveAngleMotor.set(angleSpeed);
    //swerveDriveMotor.set(speed * swerveDriveSpeedLimiter);
    
  }


  //Get the encoder value of the drive motor
  public double getDriveEncoderPosition(){

    return swerveDriveMotor.getSelectedSensorPosition();

  }

  //Zeros the encoder for the drive motor
  public void zeroEncoder(){

    swerveDriveMotor.setSelectedSensorPosition(0);
  }

  
}
