
package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.ControlConstants.*;

import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.ExtraClasses.PhotoElecSensor;
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
  private final ShooterAngle shooterAngle = new ShooterAngle();
  private final Pneumatics pneumatic = new Pneumatics();
  private final Processor processor = new Processor();

  // Extra systems
  private final NetworkTableQuerier table = new NetworkTableQuerier();
  private final LED led = new LED();
  private PhotoElecSensor photoSensor = new PhotoElecSensor();


  //===COMMANDS===//

  //Driving Commands
  //private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final FieldDriveWithJoysticks fieldDriveCommand = new FieldDriveWithJoysticks(swervedrivewpi,xbox);
  private final ChangeSpeedCommand changeSpeedCommand = new ChangeSpeedCommand();
  private final ChangeDriveMode changeModeCommand = new ChangeDriveMode();

  // Auto Commands
  private final AutoPickupNote autoPickupNoteCommand = new AutoPickupNote(swervedrivewpi,intake,shooter,table, 10);
  private final AutoShooterAmpPos autoShooterAmpPosCommand = new AutoShooterAmpPos(shooterAngle,shooter,processor,intake,5.0);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 
  
  //LED Command
  private final LEDCommand ledCommand = new LEDCommand(led);

  //Shooter Command
  private final RunShooterAmp ampShooterCommand = new RunShooterAmp(shooter);
  private final RunShooterSpeaker speakerShooterCommand = new RunShooterSpeaker(shooter, processor,intake,5);
  private final AutoShooterSpeed shooterSpeedCommand = new AutoShooterSpeed(shooter);

  //Shooter Angle Command
  private final RunAngleUp angleUpCommand = new RunAngleUp(shooterAngle);
  private final RunAngleDown angleDownCommand = new RunAngleDown(shooterAngle);
  private final AutoShooterPos autoShooterPosCommand = new AutoShooterPos(shooterAngle, table);

  //Intake Command
  private final RunIntake intakeCommand = new RunIntake(intake);
  private final TakeInNote takeInNoteCommand = new TakeInNote(intake,processor,10);

  //Climber Command
  private final RunClimber climberCommand = new RunClimber(pneumatic);

  //===BUTTONS===//

  // Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;
  private final Trigger intakeButton;
  private final Trigger speakerShootButton;
  private final Trigger ampShootButton;
  private final Trigger climberButton;
  private final Trigger runAngleDownButton;
  private final Trigger runAngleUpButton;
  
  // Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private static JoystickButton blueTeamButton;
  private static JoystickButton redTeamButton;
  private static JoystickButton parkButton;
  private static JoystickButton leftButton;
  private static JoystickButton rightButton;
  private static JoystickButton ampAngleButton;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
  
    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    intakeButton = new JoystickButton(secondaryXbox, xboxXButton);
    speakerShootButton = new JoystickButton(xbox, xboxAButton);
    ampShootButton = new JoystickButton(secondaryXbox, xboxYButton);
    climberButton = new JoystickButton(xbox, xboxBButton);
    runAngleDownButton = new JoystickButton(xbox,xboxLeftBumber);
    runAngleUpButton = new JoystickButton(xbox,xboxRightBumber);
    
    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    rightButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    leftButton = new JoystickButton(launchpad, LaunchPadSwitch6top);
    ampAngleButton = new JoystickButton(launchpad, LaunchPadSwitch1top);
  
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

    // Swerve drive default command
    swervedrivewpi.setDefaultCommand(fieldDriveCommand);

    // Shooter default command
    //shooter.setDefaultCommand(shooterSpeedCommand);

    // Shooter angle default command
    //shootAngle.setDefaultCommand(autoShooterPosCommand);

    // LED default command
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
    intakeButton.onTrue(takeInNoteCommand);
    speakerShootButton.onTrue(speakerShooterCommand);
    ampShootButton.onTrue(ampShooterCommand);
    ampAngleButton.onTrue(autoShooterAmpPosCommand);
    climberButton.onTrue(climberCommand);
    runAngleDownButton.whileTrue(angleDownCommand);
    runAngleUpButton.whileTrue(angleUpCommand);


  }

  /*
  * Sets the LED color for the selected target object
  */
  public void getColorSelection()
  {
    
    if (redTeamButton.getAsBoolean() == true) {
      
      ledColor = 0.91; //Purple
      blueAlliance = false;
 
    } else if (blueTeamButton.getAsBoolean() == true) {
      
      ledColor = 0.69; //Yellow
      blueAlliance = true;

    } else {
      
      ledColor = 0.55; //Orange
      
    }
  }

  /*
   * Determine the alliance color based on OI
   * switch position
   */
  public void getAllianceColor() 
  {

    if (redTeamButton.getAsBoolean() == true) {
      
      blueAlliance = false;
 
    } else if (blueTeamButton.getAsBoolean() == true) {
      
      blueAlliance = true;

    } else {
      
      blueAlliance = true;
      
    }

  }

  /*
   * Get the starting position in auto
   * Set by a switch on the OI
   */
  public void getAutoPosition() 
  {

    if (leftButton.getAsBoolean() == true) {
      autoPosition = "Left"; 
    } else if(rightButton.getAsBoolean() == true) {
      autoPosition = "Right";
    } else {
      autoPosition = "Center"; 
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
   * Updates the speed of the shooter motors based on which
   * AprilTags are visible
   */
  public void updateShooterSpeed() {

    // Get AprilTag status
    double tagsFound = table.getTagsFound("Cam2");

    // Determine shooter speed based on which tags are seen
    if (tagsFound > 0) {

      // Determine the closest note
      int closestTag = 0;
      double closestDistance = 9999.0;
      if(tagsFound > 1){
        for(int i = 0; i < tagsFound; i++){
          if(table.getRingInfo("Cam2",i,"distance") < closestDistance){
            closestDistance = table.getRingInfo("Cam2",i,"distance");
            closestTag = i;
          }
        }
      }

    } else {

      // No AprilTags found so set idle shooter speed
      ShooterMode = "IDLE";
      
    }

  }
  
  /*
   * Return the correct auto command to the scheduler
   */
  public Command getAutonomousCommand() {

    String autoDecision = autoPosition + Double.toString(autoNotes - 1);
    Command autoCommand = autoPickupNoteCommand;

    switch (autoDecision) {

      case "Left0":
        autoCommand = fieldDriveCommand;
        break;

      case "Left1":
        autoCommand = fieldDriveCommand;
        break;

      case "Left2":
        autoCommand = fieldDriveCommand;
        break;

      case "Left3":
        autoCommand = fieldDriveCommand;
        break;

      case "Left4":
        autoCommand = fieldDriveCommand;
        break;

      case "Left5":
        autoCommand = fieldDriveCommand;
        break;

      case "Right0":
        autoCommand = fieldDriveCommand;
        break;

      case "Right1":
        autoCommand = fieldDriveCommand;
        break;

      case "Right2":
        autoCommand = fieldDriveCommand;
        break;

      case "Right3":
        autoCommand = fieldDriveCommand;
        break;

      case "Right4":
        autoCommand = fieldDriveCommand;
        break;

      case "Right5":
        autoCommand = fieldDriveCommand;
        break;

      case "Center0":
        autoCommand = fieldDriveCommand;
        break;

      case "Center1":
        autoCommand = autoPickupNoteCommand;
        break;

      case "Center2":
        autoCommand = fieldDriveCommand;
        break;

      case "Center3":
        autoCommand = fieldDriveCommand;
        break;

      case "Center4":
        autoCommand = fieldDriveCommand;
        break;

      case "Center5":
        autoCommand = fieldDriveCommand;
        break;

      default: 
        break;
        
    }

    return autoCommand;

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

    table.putControlDouble("RobotStop", 1.0);

  }

  /*
   * Send updates on important values to dashboard
   * Called from RobotPeriodic
   */
  public void updateRobotStatus() {
    
    //Update shooter position
    SmartDashboard.putBoolean("Top Shooter", shooterAngle.getTopSwitch());
    SmartDashboard.putBoolean("Bottom Shooter", shooterAngle.getBottomSwitch());
    SmartDashboard.putNumber("Shooter Encoder", shooterAngle.getIntegratedValue());
    SmartDashboard.putNumber("Shooter Angle", CurrentShooterAngle);
    SmartDashboard.putNumber("Last Angle", LastShooterAngle);
    SmartDashboard.putBoolean("Can Shoot", readyToShoot);

    //Update Photo Sensor
    photoSensor.isNoteOnBoard();
    SmartDashboard.putBoolean("Note On Board", noteOnBoard);

    //Update Ready to Shoot
    SmartDashboard.putBoolean("Ready To Shoot", readyToShoot);

    //Update Climber Position
    SmartDashboard.putBoolean("Climb Extended", ClimberExtended);

    //Update Gyro Position
    SmartDashboard.putNumber("Gyro Angle", swervedrivewpi.getGyroAngle());

    //Update Pressure
    SmartDashboard.putNumber("Pressure", pneumatic.getPressure());

    //Update Slow Mode
    SmartDashboard.putBoolean("Slow Mode", isSlowMode);

  }

}