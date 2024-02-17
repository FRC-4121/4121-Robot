/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.ExtraClasses.Gains;
import com.revrobotics.CANSparkBase.IdleMode;


/**
 * The Constants class provides a dope af place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {


    /*
     * Values used for driving
     */
    public static final class DriveConstants {

        /*
         * Swerve drive values
         */

        // Swerve drive motors CAN IDs
        public static final int kLeftFrontDrive = 1;
        public static final int kLeftFrontAngle = 2;
        public static final int kLeftFrontCoder = 3;
        public static final int kRightFrontDrive = 4;
        public static final int kRightFrontAngle = 5;
        public static final int kRightFrontCoder = 6;
        public static final int kRightBackDrive = 7;
        public static final int kRightBackAngle = 8;
        public static final int kRightBackCoder = 9;
        public static final int kLeftBackDrive = 10;
        public static final int kLeftBackAngle = 11;
        public static final int kLeftBackCoder = 12;

        // Swerve drive calculation constants
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.546;
        public static final double DRIVETRAIN_TRACKWIDTH_INCH = 21.50;
        public static final double DRIVETRAIN_WHEELBASE_INCH = 21.5;
        public static final double lengthFromAxle = 23.5;
        public static final double widthFromAxle = 19.5;
        public static final double leftGyroCorrection = 63;//Gyro correction in degrees
        public static final double rightGyroCorrection = 297;//Gyro correction in degrees

        // Swerve drive PID constants
        public static final double drivePIDkPs[] = { 0.0, 0.0, 0.0, 0.0 };
        public static final double drivePIDkIs[] = { 0.0, 0.0, 0.0, 0.0 };
        public static final double drivePIDkDs[] = { 0.0, 0.0, 0.0, 0.0 };
        public static final double drivePIDkFs[] = { 0.0454, 0.0459, 0.0458, 0.046 };
        public static final double anglePIDkPs[] = { 3.25, 3.25, 3.25, 3.25 };
        public static final double anglePIDkIs[] = { 2.25, 2.25, 2.25, 2.25 };
        public static final double anglePIDkDs[] = { 0.04, 0.04, 0.04, 0.04 };
        public static double angleLimiters[] = { 1.0, 1.0, 1.0, 1.0 };

        // Auto drive PID constants
        public static final double kAutoDrivePIDkp = 0.03;
        public static final double kAutoDrivePIDkI = 0.0;
        public static final double kAutoDrivePIDkD = 0.0;
        public static final double kAutoDrivePIDkF = 0.0;

        // Motion magic constants
        public static final int kSlotIdxDrive = 0;
        public static final int kPIDLoopIdxDrive = 0;
        public static final int kTimeoutMsDrive = 20;
        public static final double kDriveDeadband = 0.001;
        public static final double kAngleDeadband = 0.001;
        public static final double kNominalDrive = 0.0;
        public static final double kPeakDriveForward = 1.0;
        public static final double kPeakDriveReverse = -1.0;
        public static final Gains driveGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

        /*
         * West coast drive values
         */

        // West coast drive motors CAN IDs
        public static final int LEFT_MASTER_F = 3;
        public static final int LEFT_SLAVE_F = 2;
        public static final int RIGHT_MASTER_F = 1;
        public static final int RIGHT_SLAVE_F = 4;

        /**
         * Mecanum Drive Constants
         */

        // Mecanum motor CAN IDs
        public static final int LeftFrontMotorID = 1;
        public static final int LeftBackMotorID = 4;
        public static final int RightFrontMotorID = 2;
        public static final int RightBackMotorID = 3;

        // Mecanum motor limits
        public static final double MecanumSpeedLimiter = 0.8; // Can Test This

    }


    /*
     * Values used for mechanisms
     */
    public static final class MechanismConstants {

        /*
         * Shooter Angle System Constants 
         */

        // Angle motor CAN IDs
        public static final int kPivotMotorID = 15;

        // PID values
        public static final double kShooterAngleKP = 1;
        public static final double kShooterAngleKI = 0;
        public static final double kShooterAngleKD = 0;
        public static final double kShooterAngleFF = 0;
        public static final double kShooterAngleMinOutput = -1;
        public static final double kShooterAngleMaxOutput = 1;

        // Encoder Values
        public static final int kAngleEncoderID = 2;
        public static final double kDistancePerRotation = 4.0;

        // Angle Motor Configuration
        public static final IdleMode kAngleMotorIdleMode = IdleMode.kBrake;
        public static final int kAngleMotorCurrentLimit = 20;
        public static final double AngleMotorSpeed = 0.5;

        
        //Angles for certain shots
        public static final double AmpAngle = 60;//degrees, needs to be confirmed
        public static final double HighSpeakerAngle = 52;
        public static final double LowSpeakerAngle = 35;
        public static final double MaxSpeakerAngle = 60;
        public static final double MinSpeakerAngle = 30;


        //Current Shooter Angle
        public static double CurrentShooterAngle = 52;
        public static final double ShooterAngleTolerance = 1;

        //Shooter Angle Encoder
        public static double MaxEncoderPos = 100000;//Need to set

        /**
         * Pneumatic System Constants
         */

        // Pneumatics Controller IDs
        public static final int ClimberOpenChannelID = 9;
        public static final int ClimberCloseChannelID = 8;
        public static final int ControlModuleID = 61;

        //Pneumatic General Constants
        public static boolean ClimberExtended = false;
        public static double pressureSensorVoltage = 5;

        /**
         * LED String Constants
         */

        // General Constants
        public static double ledColor = 0.63; // 0.65 is orange, the default color
        public static Boolean getCone = false; // Are the led's yellow
        public static String autoPosition = "Left"; 
        public static Boolean allianceColor = true; // true = blue, red = false
        public static Boolean noteOnBoard = true;
        public static Boolean readyToShoot = false;

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

    }


    /*
     * Values used for control inputs
     */
    public static final class ControlConstants {

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

    }

    // Motor Limits
    public static final double MaxLinearSpeed = 4; //Max Speed in Meters per second
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



    



    /**
     * General Robot Constants
     */

    // General variables
    public static boolean killAuto = false;
    public static double autoNotes = 1;

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

    //Shooter variables
    public static double shooterTargetRPM = 100;// need to test to figure out
    public static double shooterActualRPM = 0;
    public static final double shooterRPMTol = 100;  // tolerance for shooter RPM
    public static boolean shootLow = true;
    public static double lidarMin = 2;
    public static double lidarMax = 140;//need to test to find
    public static double visionDistanceTolerance = 10;
    public static double defaultShooterSpeed = 0.26;
    public static final double kP_Shoot = 0.00027; //was 0.1
    public static final double kI_Shoot = 0.000025;
    public static final double kD_Shoot = 0.000055;
    public static final double kF_Shoot = -1;
    public static final double distanceCorrection = 0;//need to find
    public static final int kPIDLoopIdxShoot = 0;
    public static final int kTimeoutMsShoot = 20;
    public static final int kShooterMaxRPM = 6100;
    public static boolean toggleShooterOnOrOff = true; //true runs the shooter motors, false keeps them off.
    public static boolean OKToShoot = false;
    public static int isBallShot = 0;
    public static boolean photoSensorIsNotBlocked;
    
}