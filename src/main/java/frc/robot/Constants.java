/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.ExtraClasses.Gains;

/**
 * The Constants class provides a dope af place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    /**
     * West Coast drive constants
     */

    // Motor CAN IDs
    public static final int LEFT_MASTER_ID = 1;
    public static final int LEFT_SLAVE_ID = 2;
    public static final int RIGHT_MASTER_ID = 3;
    public static final int RIGHT_SLAVE_ID = 4;
    
    /**
     * Shooter Constants
     */

     // Motor CAN IDs
     public static final int TOP_SHOOTER_ID = 5;
     public static final int BOTTOM_SHOOTER_ID = 6;

    // Speed Constants
    public static final double TOP_SHOOTER_SPEED = 1.0;
    public static final double BOTTOM_SHOOTER_SPEED = 1.0;

    /**
     * Gamepad Constants
     */

    // Gamepad port IDs
    public static final int XBOX_PORT = 0;

    // Gamepad button IDS
    public static final int xboxAButton = 1;
    public static final int xboxBButton = 2;
    public static final int xboxXButton = 3;
    public static final int xboxYButton = 4;
    public static final int xboxLeftBumber = 5;
    public static final int xboxRightBumber = 6;
    public static final int xboxBackButton = 7;// this button is in the middle of the xbox controller
    public static final int xboxStartButton = 8;// this button is in the middle of the xbox controller
    public static final int xboxLeftJoystickButton = 9;
    public static final int xboxRightJoystickButton = 10;


    /**
     * Operator Interface Constants
     */

    // OI Button IDs
    public static final int LaunchPadButton1 = 7;
    public static final int LaunchPadButton2 = 17;
    public static final int LaunchPadButton3 = 19;
    public static final int LaunchPadButton4 = 18;
    public static final int LaunchPadSwitch1bottom = 1;
    public static final int LaunchPadSwitch1top = 2;
    public static final int LaunchPadSwitch2bottom = 3;
    public static final int LaunchPadSwitch2top = 4;
    public static final int LaunchPadSwitch3 = 5;
    public static final int LaunchPadSwitch4 = 6;
    public static final int LaunchPadSwitch5bottom = 8;
    public static final int LaunchPadSwitch5top = 9;
    public static final int LaunchPadSwitch6bottom = 10;
    public static final int LaunchPadSwitch6top = 11;
    public static final int LaunchPadSwitch7 = 12;
    public static final int LaunchPadSwitch8 = 13;
    public static final int LaunchPadDial1 = 14; // low bit
    public static final int LaunchPadDial2 = 15;
    public static final int LaunchPadDial3 = 16; // high bit



    /**
     * General Robot Constants
     */

    // General variables
    public static boolean killAuto = false;
    public static int ballsOnBoard = 1;

    public static final boolean kMotorInvert = true;// True -> right side motors are inverted
    public static final double kTalonFXPPR = 2048;
    public static final double kWheelDiameter = 3.75;
    public static final double kLowGearSpeedCap = 0.8;// In case full speed draws excessive power, these are an
                                                      // emergency measure
    public static final double kHighGearSpeedCap = 1.0;
    public static final double kDriveGearRatio = 8.14;
    public static final double kManualDriveSpeed = 0.75;
    public static final double kAutoDriveSpeed = 0.5;
    public static final double kAutoDriveSpeedMin = 0.25;
    public static final double kAutoShootDriveSpeed = 0.75;
    public static final double kAutoTurnSpeed = 0.5;
    // public static final double kLowGearRatio = 30.0;
    // public static final double kHighGearRatio = 70.0;
    public static final double kGearRatio = 7;
    public static final double kTurnAngleTolerance = 0.001;
    public static final double kDriveDistanceTolerance = 10.0;
    public static final double AUTO_ENCODER_REVOLUTION_FACTOR = 14750.0;
    public static Boolean runAutoSpeedControl = true;

    public static final double kP_Straight = 0.012; // was 0.024
    public static final double kI_Straight = 0.0;
    public static final double kD_Straight = 0.0;
    public static final double kP_Turn = .008;// was .002
    public static final double kI_Turn = 0.0;
    public static final double kD_Turn = 0.0015;// was 0.0004
    
    public static final double kP_DriveAngle = 11.0;
    public static final double kI_DriveAngle = 8.0;
    public static final double kD_DriveAngle = 0.04;

    public static final double kSpeedCorrection = 0.9; // this will be used to compensate for differnces in the drive
                                                       // motors

    // Filtering (for gyro)
    public static final int FILTER_WINDOW_SIZE = 10;

    public static int DIRECTION_MULTIPLIER = 1;// Controls whether forward on joysticks is forward or backward on robot

    public static double kLowGearMultiplier = 0.40;
    public static double kHighGearMultiplier = 0.70;
    public static double currentGear = kHighGearMultiplier;
}