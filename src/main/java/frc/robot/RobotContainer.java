
package frc.robot;

import static frc.robot.Constants.*;

import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
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
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  // Extra systems
  private final NetworkTableQuerier table = new NetworkTableQuerier();
  private final LED led = new LED();


  //===COMMANDS===//

  //Driving Commands
  //private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final FieldDriveWithJoysticks fieldDriveCommand = new FieldDriveWithJoysticks(swervedrivewpi,xbox);
  private final ChangeSpeedCommand changeSpeedCommand = new ChangeSpeedCommand();
  private final ChangeDriveMode changeModeCommand = new ChangeDriveMode();

  // Auto Commands

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 
  
  //LED Command
  private final LEDCommand ledCommand = new LEDCommand(led);

  //Shooter Command
  private final RunShooterAmp ampShooterCommand = new RunShooterAmp(shooter);
  private final RunShooterSpeaker speakerShooterCommand = new RunShooterSpeaker(shooter);

  private final RunIntake intakeCommand = new RunIntake(intake);

  //===BUTTONS===//

  // Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;
  private final Trigger intakeButton;
  private final Trigger speakerShootButton;
  private final Trigger ampShootButton;
  
  // Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private static JoystickButton yellowButton;
  private static JoystickButton purpleButton;
  private static JoystickButton parkButton;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
    
    // Initialize Color Buttons
    yellowButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    purpleButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
  
    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxXButton);
    changeModeButton = new JoystickButton(xbox, xboxYButton);
    intakeButton = new JoystickButton(secondaryXbox, xboxXButton);
    speakerShootButton = new JoystickButton(secondaryXbox, xboxAButton);
    ampShootButton = new JoystickButton(secondaryXbox, xboxYButton);
    
    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);

    // Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    //Make sure the positions are zero
    zeroRobot();

    // Start driver cameras
    startDriverCams();

  }


  //===METHODS, WHERE STUFF IS CONFIGURED===///

  /*
   * Set default commands for all subsystems
   */
  private void configureDefaultCommands() {

    swervedrivewpi.setDefaultCommand(fieldDriveCommand);

    //LED default command
    led.setDefaultCommand(ledCommand);

  }
  
  /*
   * Assign commands to button actions
   */
  private void configureButtonBindings() {

    // Auto Commands
    killAutoButton.onTrue(killAutoObject);
    killAutoButton.onFalse(killAutoObject);

    // Teleop Commands
    changeSpeedButton.onTrue(changeSpeedCommand);
    changeModeButton.onTrue(changeModeCommand);
    intakeButton.onTrue(intakeCommand);
    speakerShootButton.onTrue(speakerShooterCommand);
    ampShootButton.onTrue(ampShooterCommand);

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
    //CameraServer.startAutomaticCapture("Arm Cam", 0);
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
    
  }

  /*
   * Send a signal to stop the Pi codes
   */
  public void stopPi() {

    table.putVisionDouble("RobotStop", 1.0);

  }

}