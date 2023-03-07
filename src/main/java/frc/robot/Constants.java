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

    // Talon SRX and FX IDs (must be unique, may range from 0+)
    // Drivetrain motor IDs
    public static final int LEFT_MASTER_F = 3;
    public static final int LEFT_SLAVE_F = 2;
    public static final int RIGHT_MASTER_F = 1;
    public static final int RIGHT_SLAVE_F = 4;

    // Arm
    public static final int Extend = 15; 
    public static final int Rotate1 = 16; 
    public static final int Rotate2 = 17;
    public static final int HomeSwitchID = 0;
    public static final int ExtendSwitchID = 1;
    public static Boolean runAutoArmExtend = false;
    public static double armTargetEncoder = 0.0; //This is 0 to start with because we don't want the arm to move.
    public static final double autoArmkP = 0.01;
    public static final double autoArmkI = 0.01;
    public static final double autoArmkD = 0.01;
    public static final double teleopRotateSpeed = 0.1;
    public static final double autoRotateSpeed = 0.3;
    public static final double autoExtendSpeed = 0.45;
    public static final double rotateRampRate = 0.07;
    public static final double rotateSlope = 1;
    public static final double rotateIntercept = 0;
    public static final double extendSlope = 1;
    public static final double extendIntercept = 0;
    public static final Gains rotateGains = new Gains(0.2,0.0,0.0,0.2,0,1.0);
    public static final Gains extendGains = new Gains(0.2,0.0,0.0,0.2,0,1.0);
    public static final double rotateVelocity = 0.0150;
    public static final double rotateAcceleration = 0.0060;
    public static final double RotateStartAngle = 0;
    public static final double RotateFloorAngle = -130000;
    public static final double RotateMidAngle = -38000; 
    public static final double RotateHighAngle = -30000;
    public static final double ExtendStartLength = 0; 
    public static final double ExtendFloorLength = 71000; 
    public static final double ExtendMidLength = 111000; 
    public static final double ExtendHighLength = 241000; 

    // Wrist
    public static final int WristID = 18; 

    // Grabber
    public static final int Intake = 21;
    public static Boolean Grabbed = false;

    //Pneumatics
    public static final int GrabOpenChannelID = 8;
    public static final int GrabCloseChannelID = 14;
    public static final int BrakeOpenChannelID = 13;
    public static final int BrakeCloseChannelID = 15;
    public static final int ControlModuleID = 61;

    // LED's
    public static double ledColor = 0.63; //0.65 is orange, the default color

    // Drive control port IDs
    public static final int XBOX_PORT = 0;

    // Xbox controller button IDS
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

    // LaunchPad button IDs
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

    // Sensor port IDs
    public static final int LIDAR_PORT = 0;

    // General variables
    public static boolean killAuto = false;
    public static int ballsOnBoard = 1;
    public static final double kCameraCorrection = 3.5;

    public static final boolean kMotorInvert = true;// True -> right side motors are inverted
    public static final int kPIDLoopIdxDrive = 0;
    public static final int kTimeoutMsDrive = 20;
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


    // Swerve drive constants
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.546;
    public static final double DRIVETRAIN_TRACKWIDTH_INCH = 21.50;
    public static final double DRIVETRAIN_WHEELBASE_INCH = 21.5;
    public static final double swerveDriveSpeedLimiter = 0.6;
    public static final double autoSwerveDriveSpeedLimiter = 0.6;
    public static final double autoSwerveDriveAngleLimiter = 2.0;

    public static final int LeftFrontDrive = 1;
    public static final int LeftFrontAngle = 2;
    public static final int LeftFrontCoder = 3;

    public static final int RightFrontDrive = 4;
    public static final int RightFrontAngle = 5;
    public static final int RightFrontCoder = 6;

    public static final int RightBackDrive = 7;
    public static final int RightBackAngle = 8;
    public static final int RightBackCoder = 9;

    public static final int LeftBackDrive = 10;
    public static final int LeftBackAngle = 11;
    public static final int LeftBackCoder = 12;

    public static final double kJoystickSpeedCorr = 1;

    public static final double lengthFromAxle = 23.5;
    public static final double widthFromAxle = 19.5;

    public static final double MaxVoltsMK4 = 12.0; // max voltage of swerve module

    public static boolean isParked = false;

    public static final double drivePIDkPs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkIs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkDs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkFs[] = { 0.0454, 0.0459, 0.0458, 0.046 };

    public static final double anglePIDkPs[] = { 3.25, 3.25, 3.25, 3.25 };
    public static final double anglePIDkIs[] = { 2.25, 2.25, 2.25, 2.25 };
    public static final double anglePIDkDs[] = { 0.04, 0.04, 0.04, 0.04 };
    public static final double angleLimiters[] = { 1.0, 1.0, 1.0, 1.0 };


}