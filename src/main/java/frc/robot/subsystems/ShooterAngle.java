// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.filter.MedianFilter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterAngle extends SubsystemBase {

  // Motors
  // private CANSparkMax pivotMotor;
  private TalonFX pivotMotor;

  // Duty Cycle Encoder (for use w/ Rev Through Bore Encoder)
  public final DutyCycleEncoder encoder;

  // Limit switches to know when we have rotated all the way down or all the way
  // up
  private DigitalInput TopSwitch = new DigitalInput(0);
  private DigitalInput BottomSwitch = new DigitalInput(9);

  // Median Filter
  private MedianFilter angle_filter;

  private double currentAngle = 55;
  private double maxEncoderPos = 27300;
  // private double currentEncoder = 0;

  private static final int pivotMotorId = 15;
  public static final int angleEncoderId = 2;
  public static final double distancePerRotation = 4.0;
  private static final double configTimeout = 0.02;
  private static final double kP = 0.000085;
  private static final double kI = 0.000001;
  private static final double kD = 0.000009;
  
  public static final double maxSpeakerAngle = 55;
  public static final double minSpeakerAngle = 32;

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {

    // Create a new Spark MAX controller for shooter angle
    // pivotMotor = new CANSparkMax(kPivotMotorID,MotorType.kBrushless);
    pivotMotor = new TalonFX(pivotMotorId);
    var configs = pivotMotor.getConfigurator();
    {
      StatusCode code = configs
          .apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor), configTimeout);
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring feedback for angle motor failed with code: %s", code),
            false);
      }
    }
    {
      StatusCode code = configs.apply(
          new MotorOutputConfigs()
              .withInverted(InvertedValue.Clockwise_Positive) // Clockwise positive for this motor.
              .withNeutralMode(NeutralModeValue.Brake), // When we aren't writing a value to the motor, we want to
                                                        // brake.
          configTimeout);
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring output levels for angle motor failed with code: %s", code),
            false);
      }
    }
    {
      StatusCode code = configs.apply(
          new Slot0Configs()
              .withKP(kP)
              .withKI(kI)
              .withKD(kD),
          configTimeout);
      if (code != StatusCode.OK) {
        DriverStation.reportError(String.format("Configuring PID for angle motor failed with code: %s", code),
            false);
      }
    }

    // Factory reset so we can get the Spark MAX to a know state before
    // configuring. Useful if we swap out a controller.
    // pivotMotor.restoreFactoryDefaults();

    // Set brake mode and current limit
    // pivotMotor.setIdleMode(kAngleMotorIdleMode);
    // pivotMotor.setSmartCurrentLimit(kAngleMotorCurrentLimit);

    // Create new Duty Cycle Encoder
    encoder = new DutyCycleEncoder(angleEncoderId);
    // encoder.setDistancePerRotation(distancePerRotation);
    encoder.setDistancePerRotation(100.0);

    angle_filter = new MedianFilter(Constants.FILTER_WINDOW_SIZE);

  }

  /**
   * 
   * Run the angle motor at a given speed
   * 
   * @param speed Angle motor speed
   * 
   */
  public void runPivot(double speed) {
    if (currentAngle >= 50)
      pivotMotor.set(speed * 0.5);
    else
      pivotMotor.set(speed);

    if (TopSwitch.get() == true) {
      if (speed > 0)
        pivotMotor.set(0);
      pivotMotor.setPosition(0);
    } else if (BottomSwitch.get() == true) {
      if (speed < 0) {
        pivotMotor.set(0);
      }
      maxEncoderPos = getIntegratedValue();
    }

    currentAngle = getCurrentAngle();
  }

  /**
   * 
   * Run the shooter to a specified angle
   * The motor controller PID controls the position
   * 
   * @param angle Target angle
   * 
   */
  public void runPivotToAngle(double angle) {
    double targetPosition = -getEncoderForAngle(angle);
    pivotMotor.setControl(new PositionVoltage(targetPosition));

    currentAngle = getCurrentAngle();
    // currentEncoder = getIntegratedValue();
  }

  /**
   * 
   * Read the value of the top limit switch
   * Switch = True will set encoder to zero and angle to max
   * 
   */
  public boolean getTopSwitch() {
    return TopSwitch.get();
  }

  /**
   * 
   * Read the value of the bottom limit switch
   * Switch = True will set encoder to max and angle to min
   * 
   */
  public boolean getBottomSwitch() {
    return BottomSwitch.get();
  }

  /**
   * 
   * Read the absolute encoder raw value
   * 
   */
  public double getEncoderValue() {
    return Math.abs(encoder.get());
  }

  /**
   * 
   * Read the absolute encoder position
   * 
   */
  public double getAbsoluteEncoderPosition() {
    return encoder.getAbsolutePosition();
  }

  /**
   * 
   * Read the integrated encoder absolute distance and average it
   */
  public double getEncoderDistance() {
    return angle_filter.calculate(Math.abs(encoder.getDistance()));
  }

  /**
   * 
   * Read the integrated encoder position and average it
   * 
   */
  public double getIntegratedValue() {
    return (int) Math.round(angle_filter.calculate(Math.abs(pivotMotor.getPosition().refresh().getValue())));
  }

  /**
   * 
   * Zero the integrated encoder
   * 
   */
  public void zeroEncoder() {
    pivotMotor.setPosition(0, configTimeout);
  }

  /**
   * 
   * Calculate the current shooter angle from the motor encoder position
   * 
   */
  public double getCurrentAngle() {
    return getIntegratedValue() * ((minSpeakerAngle - maxSpeakerAngle) / maxEncoderPos) + maxSpeakerAngle;
  }

  /**
   * 
   * Calculates the encoder value for a given angle
   * 
   * @param angle Angle to calculate encoder position for
   * 
   */
  public double getEncoderForAngle(double angle) {
    return Math.round(maxEncoderPos * (angle - maxSpeakerAngle) / (minSpeakerAngle - maxSpeakerAngle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
