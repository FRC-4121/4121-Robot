// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class MecanumDrivetrain extends SubsystemBase {
  
  //Define motor controllers
  private WPI_TalonSRX frontLeftMotor;
  private WPI_TalonSRX frontRightMotor;
  private WPI_TalonSRX backLeftMotor;
  private WPI_TalonSRX backRightMotor;
  private MecanumDrive mecanumDrive;

  //Define local variables
  private double gyroAngle;
  private double speedX, speedY, speedZ;


  /** Creates a new Mecanum Drive. */
  public MecanumDrivetrain() {

    

    // Initialize motor controllers
    frontLeftMotor = new WPI_TalonSRX(LeftFrontMotorID);
    frontRightMotor = new WPI_TalonSRX(RightFrontMotorID);
    backLeftMotor = new WPI_TalonSRX(LeftBackMotorID);
    backRightMotor = new WPI_TalonSRX(RightBackMotorID);

    // Invert back motors
    backLeftMotor.setInverted(InvertType.InvertMotorOutput);
    backRightMotor.setInverted(InvertType.InvertMotorOutput);

    // Initialize mecanum drive
    mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

   
  }
  
  public void drive(double leftJoyX, double leftJoyY, double rightJoyX, boolean useGyro)
  {
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    //Set properties of drive
    mecanumDrive.setSafetyEnabled(false);	
    mecanumDrive.setMaxOutput(0.95);
    
    //Get joystick values and scale
    speedX = -leftJoyY * MecanumSpeedLimiter;
    speedY = leftJoyX * MecanumSpeedLimiter;
    speedZ = rightJoyX * MecanumSpeedLimiter;

    //If using the pi gyro board, mod the returned value by 360 to avoid a spinning robot
    if(useGyro) {

      //gyroAngle = Robot.driveAngle.getDouble(0) % 360.0;
  
      mecanumDrive.driveCartesian(speedY, speedX, speedZ, new Rotation2d(0.0));
  
    } else {

      mecanumDrive.driveCartesian(speedY, speedX, speedZ);

    }

    // SmartDashboard.putString("Gyro Angle", Double.toString(gyroAngle));
    // SmartDashboard.putString("Gyro Yaw", Double.toString(Robot.gyroYaw.getDouble(0)));

    SmartDashboard.putNumber("Front Left Drive", frontLeftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Back Left Drive", backLeftMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Front Right Drive", frontRightMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Back Right Drive", backRightMotor.getSupplyCurrent());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
