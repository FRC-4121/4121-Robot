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
    public static final int LEFT_MASTER_F = 3;
    public static final int LEFT_SLAVE_F = 2;
    public static final int RIGHT_MASTER_F = 1;
    public static final int RIGHT_SLAVE_F = 4;


    /**
     * Swerve drive constants
     */

    // Swerve Calculation Constants
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.546;
    public static final double DRIVETRAIN_TRACKWIDTH_INCH = 21.50;
    public static final double DRIVETRAIN_WHEELBASE_INCH = 21.5;
    public static final double lengthFromAxle = 23.5;
    public static final double widthFromAxle = 19.5;

    // Motor Limits
    public static final double MaxLinearSpeed = 5; //Max Speed in Meters per second
    public static final double MaxRotationalSpeed = 1.5; //Max Speed in Radians per second, about pi/2 or 90 degrees
    public static final double MaxRadiansPerSecond = Math.PI;
    public static final double SlowMaxLinearSpeed = 1;//Max Speed during slow mode in meters per second
    public static final double SlowRadiansPerSecond = (Math.PI/3);//Max rotational speed during slow mode
    public static final double slowSpeed = 0.3;
    public static final double driveSpeed = 0.7;
    public static  double swerveDriveSpeedLimiter = 0.7;
    public static double LinearSpeed = 5;
    public static double RotationalSpeed = Math.PI;
    public static final double autoSwerveDriveSpeedLimiter = 0.6;
    public static final double autoSwerveDriveAngleLimiter = 2.0;
    public static final double slowAngleSpeed = 0.5;
    public static final double angleSpeed = 1.0;
    public static final double MaxVoltsMK4 = 12.0; // max voltage of swerve module

    // General Constants
    public static final double kJoystickSpeedCorr = 1;
    public static double startingPitch = 23;
    public static boolean isParked = false;
    public static boolean isFieldOriented = true;

    // Motion Magic Constants
    public static final int kSlotIdxDrive = 0;
    public static final int kPIDLoopIdxDrive = 0;
    public static final int kTimeoutMsDrive = 20;
    public static final double kDriveDeadband = 0.001;
    public static final double kAngleDeadband = 0.001;
    public static final double kNominalDrive = 0.0;
    public static final double kPeakDriveForward = 1.0;
    public static final double kPeakDriveReverse = -1.0;
    public static final Gains driveGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

    // PID Constants
    public static final double drivePIDkPs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkIs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkDs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkFs[] = { 0.0454, 0.0459, 0.0458, 0.046 };
    public static final double anglePIDkPs[] = { 3.25, 3.25, 3.25, 3.25 };
    public static final double anglePIDkIs[] = { 2.25, 2.25, 2.25, 2.25 };
    public static final double anglePIDkDs[] = { 0.04, 0.04, 0.04, 0.04 };
    public static double angleLimiters[] = { 1.0, 1.0, 1.0, 1.0 };


    /**
     * Mecanum Drive Constants
     */

    // Motor CAN IDs
    public static final int LeftFrontMotorID = 1; 
    public static final int LeftBackMotorID = 4; 
    public static final int RightFrontMotorID = 2; 
    public static final int RightBackMotorID = 3; 

    // Motor Limits
    public static final double MecanumSpeedLimiter = 0.8; //Can Test This


    /**
     * Rotatable Arm Constants
     */

    // Motor CAN IDs
    public static final int Extend = 15; 
    public static final int Rotate1 = 16; 
    public static final int Rotate2 = 17;
    public static final int HomeSwitchID = 3;

    // PID Constants
    public static final double autoArmkP = 2.0;
    public static final double autoArmkI = 0.0;
    public static final double autoArmkD = 0.0;
    public static final double autoArmExtendkP = 2.0;
    public static final double autoArmExtendkI = 0.0;
    public static final double autoArmExtendkD = 0.0;
    public static final Gains rotateGains = new Gains(0.2,0.0,0.0,0.2,0,1.0);
    public static final Gains extendGains = new Gains(0.2,0.0,0.0,0.2,0,1.0);

    // Speed Limits
    public static final double teleopRotateSpeed = 0.28;
    public static final double autoRotateSpeed = 0.28;
    public static final double autoExtendSpeed = 0.85;
    public static final double rotateRampRate = 0.07;
    public static final double rotateSlope = 1;
    public static final double rotateIntercept = 0;
    public static final double extendSlope = 1;
    public static final double extendIntercept = 0;
    public static final double rotateVelocity = 0.0150;
    public static final double rotateAcceleration = 0.0060;
    public static final double ExtendTolerance = 3000;

    // Encoder Targets
    public static final double RotateStartAngle = 0;
    public static final double RotateTravelAngle = -120000; 
    public static final double RotateFloorAngle = -121000;
    public static final double RotateMidAngle = -33000; 
    public static final double RotateHighAngle = -24000; // was -18000
    public static final double RotateHighAngleAuto = -27000;
    public static final double RotateLoadAngle = -20000;
    public static final double ExtendStartLength = 500; 
    public static final double ExtendTravelLength = 1000; 
    public static final double ExtendFloorLength = 63500; 
    public static final double ExtendMidLength = 111000; 
    public static final double ExtendHighLength = 240500; 
    public static final double ExtendLoadLength = 71000;
    public static Boolean runAutoArmExtend = false;
    public static double armTargetEncoder = 0.0; //This is 0 to start with because we don't want the arm to move.

    /**
     * Rotatable Wrist Constants
     */

    // Motor CAN IDs
    public static final int WristID = 21; // was 18

    // PID Constants
    public static final double wrist_kP = 1.0;
    public static final double wrist_kI = 0.0;
    public static final double wrist_kD = 0.0;
    public static final double wrist_kIz = 0.0;
    public static final double wrist_kFF = 0.0;

    // Motor Limits
    public static final double wrist_MaxOutput = 1.0;
    public static final double wrist_MinOutput = -1.0;
    public static final double wristSpeed = 0.6;
    public static final double wristRotateLimit = -86.0;

    // Encoder Targets
    public static final double wristSlopeDownEmpty = (1/4.8); //The wrist takes 4.8 seconds for a full rotation down
    public static final double wristSlopeUpEmpty = (1/4.875); //It takes 5 seconds to go from the bottom to the top
    public static final double wristSlopeDownCone = (1/5.0);
    public static final double wristSlopeUpCone = (1/4.95);
    public static final double wristIntercept = 0; //Derived from linear equation testing how long it takes to get to positions
    public static final double WristTravelPosition = -5.0; //Need to confirm
    public static final double WristFloorPosition = -18.0; //Need to confirm
    public static final double WristMidPosition = -44.0; //Need to confirm
    public static final double WristHighPosition = -54.0; //Need to confirm
    public static final double WristHighPositionAuto = -49.0;
    public static final double WristLoadPosition = -60.0; //Need to confirm
    public static final double WristHomePosition = -5.0;

    // General Constants
    public static boolean GrabbedCone = true; //This is true if we are closed, becuase we can assume that we have a cone
    public static double currentWristPosition = 0.0; //Value from 0.0 to 1.0, relative position of wrist


    /**
     * Grabber Constants
     */

    // Motor CAN IDs
    public static final int Intake = 18;  // was 21

    // General Constants
    public static Boolean Grabbed = false;


    /**
     * Pneumatic System Constants
     */

    // Pneumatics Controller IDs
    public static final int GrabOpenChannelID = 7;
    public static final int GrabCloseChannelID = 5;
    public static final int BrakeOpenChannelID = 6;
    public static final int BrakeCloseChannelID = 4;
    public static final int ControlModuleID = 61;


    /**
     * LED String Constants
     */

    // General Constants
    public static double ledColor = 0.63; //0.65 is orange, the default color
    public static Boolean getCone = false; //Are the led's yellow


    /**
     * Vision System Constants
     */

    // General Constants
    public static final double targetCubeOffset = 5.8;
    public static final double targetConeOffset = 8.2;
    public static final double targetTapeOffsetLow = -8.0;
    public static final double targetTapeOffsetHigh = -7.7;
    public static final double targetTapeDistLow = 33.3;
    public static final double targetTapeDistHigh = 47.9;
    public static final double visionTolerance = 0.5;
    public static final double visionTapeTolerance = 0.2;
    public static final double kCameraCorrection = 3.5;


    /**
     * LIDAR Sensor Constants
     */

    // RoboRio Port ID
    public static final int LIDAR_PORT = 0;

    
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