
package frc.robot;

import static frc.robot.Constants.*;

import org.opencv.core.Mat;

import frc.robot.subsystems.*;
import frc.robot.subsystems.cameras.CameraBuilder;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
//import frc.robot.extraClasses.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;



public class RobotContainer {
  
  // Camera
  VideoSink camSink;
  VideoSource camSource;

  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final XboxController secondaryXbox = new XboxController(1);
  private final Joystick launchpad = new Joystick(2);
  //private final Joystick testbed = new Joystick(3);
  

  //Subsystems
  //private final SwerveDrive swervedrive = new SwerveDrive();
  private final SwerveDriveWPI swervedrivewpi = new SwerveDriveWPI();


  //private final MecanumDrivetrain mecanumDrive = new MecanumDrivetrain();

  private final NetworkTableQuerier table = new NetworkTableQuerier();

 
  //private final Pneumatics pneumatic = new Pneumatics();

  private final LED led = new LED();

  //===COMMANDS===//

  //Driving Commands
  //private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final FieldDriveWithJoysticks fieldDriveCommand = new FieldDriveWithJoysticks(swervedrivewpi,xbox);
  //private final MecanumDriveWithJoysticks mecanumDriveCommand = new MecanumDriveWithJoysticks(mecanumDrive, xbox, table);
  //private final ParkCommand parkCommand = new ParkCommand(swervedrive);
  private final ChangeSpeedCommand changeSpeedCommand = new ChangeSpeedCommand();

  // Auto Commands
  //private final AutoDrive autoDriveCommand = new AutoDrive(swervedrive, 0.6, 120, 180, 0, 0, 20, table);
  // private final AutoBalance autoBalanceCommand = new
  // AutoBalance(swervedrive,0.25,0,20,table);
  //private final AutoGroup1 autoGroup = new AutoGroup1(swervedrive, table);
  //private final AutoAlignToTape autoAlignToTape = new AutoAlignToTape(swervedrive, 0.1, 20, table);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 
  
  //Grabber Commands
  //private final Grab grab = new Grab(pneumatic);
 

  //LED Command
  private final LEDCommand ledCommand = new LEDCommand(led);

  //Brake Commands
  //private final ApplyBrake applyBrakeCommand = new ApplyBrake(pneumatic);
  //private final ReleaseBrake releaseBrakeCommand = new ReleaseBrake(pneumatic);

  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  // private final JoystickButton extendArmButton;
  // private final JoystickButton retractArmButton;
  // private final JoystickButton rotateArmForwardButton;
  // private final JoystickButton rotateArmBackwardButton;
  // private final JoystickButton wristForwardButton;
  // private final JoystickButton wristBackwardButton;
  // private final JoystickButton grabberBackwardButton;
  // private final JoystickButton grabberForwardButton;
  // private final JoystickButton grabButton;
  // private final JoystickButton letGoButton;

  private final Trigger extendArmButton;
  private final Trigger retractArmButton;
  private final Trigger rotateArmUpButton;
  private final Trigger rotateArmDownButton;
  private final Trigger wristUpButton;
  private final Trigger wristDownButton;
  private final Trigger grabberBackwardButton;
  private final Trigger grabberForwardButton;
  private final Trigger grabButton;
  private final Trigger letGoButton;
  //private final Trigger applyBrakeButton;
  //private final Trigger releaseBrakeButton;
  private final Trigger autoArmLoadButton;
  private final Trigger changeSpeedButton;
  private final Trigger zeroWristButton;
  
  //launchpad buttons/switches
  //private final JoystickButton killAutoButton;
  private final Trigger killAutoButton;
  private final Trigger autoArmTravelButton;
  private final Trigger autoArmFloorButton;
  private final Trigger autoArmMidButton;
  private final Trigger autoArmHighButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton yellowButton;
  private static JoystickButton purpleButton;
  private static JoystickButton armControlButton;
  private static JoystickButton parkButton;
  private static JoystickButton autoProg1;
  private static JoystickButton autoProg2;
  private static JoystickButton autoSel;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
    
    // colorButtons
    yellowButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    purpleButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
  
    //xboxButtons
    extendArmButton = new JoystickButton(xbox,xboxRightBumber); 
    retractArmButton = new JoystickButton(xbox,xboxLeftBumber); 
    wristUpButton = new JoystickButton(secondaryXbox,xboxAButton);
    wristDownButton = new JoystickButton(secondaryXbox,xboxBButton);
    grabberForwardButton = new JoystickButton(secondaryXbox,xboxXButton);
    grabberBackwardButton = new JoystickButton(secondaryXbox,xboxYButton);
    grabButton = new JoystickButton(secondaryXbox,xboxRightBumber);
    letGoButton = new JoystickButton(secondaryXbox,xboxLeftBumber);
    //applyBrakeButton = new JoystickButton(xbox, xboxXButton);
    //releaseBrakeButton = new JoystickButton(xbox, xboxYButton);
    autoArmLoadButton = new JoystickButton(xbox, xboxYButton);
    changeSpeedButton = new JoystickButton(xbox, xboxXButton);
    zeroWristButton = new JoystickButton(xbox, xboxRightJoystickButton);
    
    //Going to use triggers for these
    rotateArmDownButton = new JoystickButton(xbox,xboxAButton);
    rotateArmUpButton = new JoystickButton(xbox,xboxBButton);  

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    autoArmTravelButton = new JoystickButton(launchpad,LaunchPadSwitch1top);
    autoArmFloorButton = new JoystickButton(launchpad,LaunchPadSwitch1bottom);
    autoArmMidButton = new JoystickButton(launchpad,LaunchPadSwitch2bottom);
    autoArmHighButton = new JoystickButton(launchpad,LaunchPadSwitch2top);
    AutoPos1 = new JoystickButton(launchpad,LaunchPadDial1);
    AutoPos2 = new JoystickButton(launchpad,LaunchPadDial2);
    AutoPos3 = new JoystickButton(launchpad,LaunchPadDial3);
    yellowButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    purpleButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    armControlButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);
    autoProg1 = new JoystickButton(launchpad, LaunchPadSwitch6top);
    autoProg2 = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    autoSel = new JoystickButton(launchpad, LaunchPadSwitch7);

    //testbed buttons
    // extendArmButton = new JoystickButton(testbed,9); 
    // retractArmButton = new JoystickButton(testbed,10); 
    // wristForwardButton = new JoystickButton(testbed,5);
    // wristBackwardButton = new JoystickButton(testbed,6);
    // grabberForwardButton = new JoystickButton(testbed,3);
    // grabberBackwardButton = new JoystickButton(testbed,4);
    // grabButton = new JoystickButton(testbed,7);
    // letGoButton = new JoystickButton(testbed,8);
    // rotateArmBackwardButton = new JoystickButton(testbed,1);
    // rotateArmForwardButton = new JoystickButton(testbed,2);

    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    //Make sure the positions are zero
    zeroRobot();

    //Make sure the grabber is closed
    //pneumatic.grab();

    // Start driver cameras
    startDriverCams();

    // Initialize vision alignment help
    SmartDashboard.putBoolean("Move Right", false);
    SmartDashboard.putBoolean("Move Left", false);
    SmartDashboard.putBoolean("On Target", false);
    SmartDashboard.putBoolean("Tape Move Right", false);
    SmartDashboard.putBoolean("Tape Move Left", false);
    SmartDashboard.putBoolean("Tape On Target", false);

  }


  //===METHODS,WHERE STUFF IS CONFIGURED===///

  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands() {

    //Drivetrain -> drive with xbox joysticks
    //swervedrive.setDefaultCommand(driveCommand);
    swervedrivewpi.setDefaultCommand(fieldDriveCommand);
    //mecanumDrive.setDefaultCommand(mecanumDriveCommand);

    //LED default command
    led.setDefaultCommand(ledCommand);

    //Arm's default command is the extend control
    //arm.setDefaultCommand();
  }
  
  private void configureButtonBindings() {

    //Auto
    killAutoButton.onTrue(killAutoObject);
    killAutoButton.onFalse(killAutoObject);
    changeSpeedButton.onTrue(changeSpeedCommand);

    //teleop Commands
    //grabButton.whileTrue(grab);
    //applyBrakeButton.whileTrue(applyBrakeCommand);
    //releaseBrakeButton.whileTrue(releaseBrakeCommand);
  }

   /*
    * Sets the LED color for the selected target object
    */
  public void getColorSelection()
  {
    
    if (purpleButton.getAsBoolean() == true) {
      
      ledColor = 0.91; //Purple
      getCone = false;
 
    } else if (yellowButton.getAsBoolean() == true) {
      
      ledColor = 0.69; //Yellow
      getCone = true;

    } else {
      
      ledColor = 0.55; //Orange
      getCone = false;

    }
  }

  /*
   * Get driver selection for arm operation
   */
  public void getArmSelection()
  {

    if (armControlButton.getAsBoolean() == true)
    {
      runAutoArmExtend = false;
    } else {
      runAutoArmExtend = true;
    }

  }

  /*
   * Park and unpark the robot
   */
  public void getParkSelection()
  {

    if (parkButton.getAsBoolean() == true)
    {
      isParked = true;
      SmartDashboard.putBoolean("Robot Parked", true);
      //parkCommand.execute();
    } else{
      isParked = false;
      SmartDashboard.putBoolean("Robot Parked", false);
    }
  }
  
  /*
   * Return the correct auto command to the scheduler
   */
  public Command getAutonomousCommand() {

    // Get user input
    Boolean checkAutoProg1 = autoProg1.getAsBoolean();
    Boolean checkAutoProg2 = autoProg2.getAsBoolean();
    Boolean checkAutoSelect = autoSel.getAsBoolean();

    /*if (!checkAutoSelect) {

      if (checkAutoProg1) {

        return autoDriveCommand;//Changed this to arbitrary command, gonna need to make these actual auto commands

      } else if (checkAutoProg2) {

        return autoDriveCommand;//Changed this to arbitrary command, gonna need to make these actual auto commands

      } else {

        return autoDriveCommand;//Changed this to arbitrary command, gonna need to make these actual auto commands

      }
    } else {

      if (checkAutoProg1) {

        return autoDriveCommand;//Changed this to arbitrary command, gonna need to make these actual auto commands
      } else if (checkAutoProg2) {

        return autoAlignToTape;

      } else {

        return autoDriveCommand;//Changed this to arbitrary command, gonna need to make these actual auto commands
      }

    }
*/

    // Return selected command
    // switch(autoProg) {

    //   case 1:
    //     return autoDriveAndLower;
      
    //   case 2:
    //     return autoPlaceAndGetOut;

    //   case 3:
    //     return autoPlaceAndBalance;

    //   default:
    //     return autoDriveAndLower;
    // }

    return fieldDriveCommand;
  }

  /*
   * Start driver cameras
  */ 
  public void startDriverCams() {

    // UsbCamera grabCamera = new UsbCamera("Grab Cam", 0);
    // grabCamera.setResolution(160, 120);
    // grabCamera.setBrightness(100);
    // grabCamera.setFPS(24);
    CameraServer.startAutomaticCapture("Arm Cam", 0);
    // camSink = CameraServer.getVideo();
    // camSource = CameraServer.putVideo("Grab Cam", 160, 120);
    // camSource = new CameraBuilder(0, "Grab Cam (Camera)").res(160, 120).brightness(100).fps(24).finish();
    // camSink = new MjpegServer("Grab Cam (Server)", 1181);
    // camSink.setSource(camSource);
    SmartDashboard.putBoolean("Cam Started",true);

  }

  /*
   * Stream cameras every cycle
   */
  public void streamCams() {

    // camSource = CameraServer.putVideo("Grab Cam", 160, 120);
    
  }

  /*
   * Zero positions of all mechanisms
   */
  public void zeroRobot() {
    
    //Zero the encoders when robot starts up

    //Make sure that the wrist is starting at 0
    currentWristPosition = 0.0;

    //Put the encoder value on the smart dashboard
  
    //Put the wrist position on the dashboard on startup
    SmartDashboard.putNumber("WristPosition",currentWristPosition);

  }

  public void zeroWrist(){
    //Make sure that the wrist is starting at 0
    currentWristPosition = 0.0;

    //Put the wrist position on the dashboard on startup
    SmartDashboard.putNumber("WristPosition",currentWristPosition);
  }

  /*
   * Send a signal to stop the Pi codes
   */
  public void stopPi() {

    table.putVisionDouble("RobotStop", 1.0);

  }

  /*
   * Use vision system to check alignment of cones or cubes
   */
  public void checkTargetAlignment(){

    if (purpleButton.getAsBoolean() == true) {
      
      if(table.getVisionDouble("CubesFound") > 0 ) {
        
        SmartDashboard.putBoolean("Cubes Found", true);

        double cubeOffset = table.getVisionDouble("Cubes.0.offset") - targetCubeOffset;
        SmartDashboard.putNumber("Cube Offset", cubeOffset);

        if(cubeOffset < -visionTolerance) {
          
          SmartDashboard.putBoolean("Move Right", true);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", false);

        } else if(cubeOffset > visionTolerance) {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", true);
          SmartDashboard.putBoolean("On Target", false);

        } else {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", true);

        }
      } else {

        SmartDashboard.putBoolean("Cubes Found", false);

      }
 
    } else if (yellowButton.getAsBoolean() == true) {
      
      if(table.getVisionDouble("ConesFound") > 0 ) {
        
        SmartDashboard.putBoolean("Cones Found", true);

        double coneOffset = table.getVisionDouble("Cones.0.offset") - targetConeOffset;
        SmartDashboard.putNumber("Cone Offset", coneOffset);

        if(coneOffset < -visionTolerance ) {
          
          SmartDashboard.putBoolean("Move Right", true);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", false);

        } else if(coneOffset > visionTolerance) {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", true);
          SmartDashboard.putBoolean("On Target", false);

        } else {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", true);

        }
      } else {

        SmartDashboard.putBoolean("Cones Found", false);

      }

    } else {
        
      SmartDashboard.putBoolean("Move Right", false);
      SmartDashboard.putBoolean("Move Left", false);
      SmartDashboard.putBoolean("On Target", false);

    }
 
  }

  /*
   * Use vision system to check alignment of robot with goal tape
   */
  public void checkTapeAlignment() {

    if(table.getVisionDouble("TapesFound") > 0 ) {

      // Get correct offset based on which tape was found
      double targetTapeOffset = -7.5;
      double tapeY = table.getVisionDouble("Tapes.0.y");
      if (tapeY > 280) {
        targetTapeOffset = targetTapeOffsetLow;
      } else {
        targetTapeOffset = targetTapeOffsetHigh;
      }

      double tapeOffset = table.getVisionDouble("Tapes.0.offset");
      if(tapeOffset < targetTapeOffset - visionTolerance ) {
        
        SmartDashboard.putBoolean("Tape Move Right", true);
        SmartDashboard.putBoolean("Tape Move Left", false);
        SmartDashboard.putBoolean("Tape On Target", false);

      } else if(tapeOffset > targetTapeOffset + visionTolerance) {

        SmartDashboard.putBoolean("Tape Move Right", false);
        SmartDashboard.putBoolean("Tape Move Left", true);
        SmartDashboard.putBoolean("Tape On Target", false);

      } else {

        SmartDashboard.putBoolean("Tape Move Right", false);
        SmartDashboard.putBoolean("Tape Move Left", false);
        SmartDashboard.putBoolean("Tape On Target", true);

      }

    } else {

      SmartDashboard.putBoolean("Tape Move Right", false);
      SmartDashboard.putBoolean("Tape Move Left", false);
      SmartDashboard.putBoolean("Tape On Target", false);

    }

  }
}