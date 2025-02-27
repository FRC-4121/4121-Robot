// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Drivetrain extends SubsystemBase {

  private ADXRS450_Gyro gyro;
  private MedianFilter gyro_filter;

  private WPI_TalonSRX leftMasterFalcon;
  private WPI_TalonSRX leftSlaveFalcon;

  private WPI_TalonSRX rightMasterFalcon;
  private WPI_TalonSRX rightSlaveFalcon;

  private DifferentialDrive drivetrain;


  /** 
   * 
   * Drivetrain constructor 
   * 
   * Initialize motors, encoders, and gyro
 */
  public Drivetrain() {

    // Initialize drivetrain motors
    initTalonDrivetrain();

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    zeroGyro();

    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);
  }


  /**
   * 
   * Drivetrain periodic
   * 
   * Things that need to happen on a periodic basis as
   * the drivetrain is being used.
   * 
   */
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
   * Configure the drivetrain motors as a differential drive
   * and configure encoders for left and right sides
   * 
   */
  private void initTalonDrivetrain() {

    //Init motors, speed controller groups, and drivetrain
    leftMasterFalcon = new WPI_TalonSRX(LEFT_MASTER_ID);
    leftSlaveFalcon = new WPI_TalonSRX(LEFT_SLAVE_ID);
    leftSlaveFalcon.follow(leftMasterFalcon);

    rightMasterFalcon = new WPI_TalonSRX(RIGHT_MASTER_ID);
    rightSlaveFalcon = new WPI_TalonSRX(RIGHT_SLAVE_ID);
    rightSlaveFalcon.follow(rightMasterFalcon);

    //Set brake mode (check Phoenix Tuner software to confirm)
    // leftMasterFalcon.setNeutralMode(NeutralMode.Brake);
    // leftSlaveFalcon.setNeutralMode(NeutralMode.Brake);
    // rightMasterFalcon.setNeutralMode(NeutralMode.Brake);
    // rightSlaveFalcon.setNeutralMode(NeutralMode.Brake);

    //Invert appropriately
    leftMasterFalcon.setInverted(kMotorInvert);
    //leftSlaveFalcon.setInverted(!kMotorInvert);
    rightMasterFalcon.setInverted(!kMotorInvert);
    //rightSlaveFalcon.setInverted(kMotorInvert);
    
    drivetrain = new DifferentialDrive(leftMasterFalcon, rightMasterFalcon);
  }


  /**
   * 
   * Main teleop drive method
   * 
   * @param leftJoyY: left joystick position
   * @param rightJoyY: right joystick position
   * 
   */
  public void drive(double leftJoyY, double rightJoyY) {


    // Drive the motors
    // Direction multiplier indicates drive direction
    if (DIRECTION_MULTIPLIER == 1) {
      drivetrain.tankDrive(leftJoyY, rightJoyY); 
    }
    else{
      drivetrain.tankDrive(rightJoyY, leftJoyY);    
    }

  }


  /**
   * 
   * Run drivetrain during autonomous
   * 
   * @param leftSpeed: speed for left side motors
   * @param rightSpeed: speed for right side motors
   * 
   */
  public void autoDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }


  /** Stop the drive train */
  public void stopDrive() {

    drivetrain.tankDrive(0, 0);
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
  

  /** Invert the direction of driving */
  public void invertDirection() {

    DIRECTION_MULTIPLIER *= -1;

  }

  public void changeGears() {

    if (currentGear == kLowGearMultiplier)
    {
      currentGear = kHighGearMultiplier;
    } else{

      currentGear = kLowGearMultiplier;
    }
  }
  
  public double getGyroAngle() {

    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    return correctedGyro;

  }

}
