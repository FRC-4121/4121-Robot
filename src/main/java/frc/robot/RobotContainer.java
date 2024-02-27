
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
  private final AutoDrive autoDriveCommand = new AutoDrive(swervedrivewpi, 0.1, 110.0, 0.0, 0.0, 0.02, 10.0);
  private final AutoPickupNote autoPickupNoteCommand = new AutoPickupNote(swervedrivewpi,intake,shooter,table, 10);
  private final AutoShooterAmpPos autoShooterAmpPosCommand = new AutoShooterAmpPos(shooterAngle,shooter,processor,intake,5.0);
  private final AutoShooterPos autoShooterPosCommand = new AutoShooterPos(shooterAngle, table);
  private final Auto1NoteCenter auto1NoteCenterCommand = new Auto1NoteCenter(swervedrivewpi, shooter, processor, intake);
  private final Auto1NoteLeft auto1NoteLeftCommand = new Auto1NoteLeft(swervedrivewpi, shooter, processor, intake);
  private final Auto1NoteRight auto1NoteRightCommand = new Auto1NoteRight(swervedrivewpi, shooter, processor, intake);
  private final Auto2NoteCenter auto2NoteCenterCommand = new Auto2NoteCenter(swervedrivewpi, shooter, processor, intake);
  private final Auto2NoteLeft auto2NoteLeftCommand = new Auto2NoteLeft(swervedrivewpi, shooter, processor, intake);
  private final Auto2NoteRight auto2NoteRightCommand = new Auto2NoteRight(swervedrivewpi, shooter, processor, intake);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 
  
  //LED Command
  private final LEDCommand ledCommand = new LEDCommand(led);

  //Shooter Command
  private final RunShooterAmp ampShooterCommand = new RunShooterAmp(shooter,processor,intake,5);
  private final RunShooterSpeaker speakerShooterCommand = new RunShooterSpeaker(shooter, processor,intake,5);
  private final AutoShooterSpeed shooterSpeedCommand = new AutoShooterSpeed(shooter);

  //Shooter Angle Command
  private final RunAngleUp angleUpCommand = new RunAngleUp(shooterAngle);
  private final RunAngleDown angleDownCommand = new RunAngleDown(shooterAngle);
  

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
  private final Trigger autoShooterPosButton;
  
  // Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private static JoystickButton blueTeamButton;
  private static JoystickButton redTeamButton;
  private static JoystickButton parkButton;
  private static JoystickButton leftButton;
  private static JoystickButton rightButton;
  private static JoystickButton ampAngleButton;
  private static JoystickButton autoShooterPositionButton;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
  
    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    intakeButton = new JoystickButton(xbox, xboxAButton);
    speakerShootButton = new JoystickButton(secondaryXbox, xboxXButton);
    ampShootButton = new JoystickButton(secondaryXbox, xboxYButton);
    climberButton = new JoystickButton(xbox, xboxBButton);
    runAngleDownButton = new JoystickButton(secondaryXbox,xboxLeftBumber);
    runAngleUpButton = new JoystickButton(secondaryXbox,xboxRightBumber);
    autoShooterPosButton = new JoystickButton(secondaryXbox,xboxBButton);
    
    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    rightButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    leftButton = new JoystickButton(launchpad, LaunchPadSwitch6top);
    ampAngleButton = new JoystickButton(launchpad, LaunchPadSwitch1top);
    autoShooterPositionButton = new JoystickButton(launchpad, LaunchPadSwitch7);
  
    // Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    //Make sure the positions are zero
    zeroRobot();

    // Calibrate the gyro
    swervedrivewpi.calibrateGyro();

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
    shooterAngle.setDefaultCommand(autoShooterPosCommand);

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
    autoShooterPosButton.onTrue(autoDriveCommand);


  }

  /*
  * Sets the LED color for the selected target object
  */
  public void getColorSelection()
  {
    
    if (noteOnBoard == true) {
      
      ledColor = 0.65; //Orange
    
 
    } else {
      
      ledColor = 0.55; //Default Pattern 

    }

    led.setColor(ledColor);
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

  public void getAngleSelection()
  {
    if (autoShooterPositionButton.getAsBoolean() == true)
    {
      AutoShooterPositioning = true;
      SmartDashboard.putBoolean("Auto Positioning", true);
      //parkCommand.execute();
    } else{
      AutoShooterPositioning = false;
      SmartDashboard.putBoolean("Auto Positioning", false);
    }
  }

  /*
   * Check for the presence of a note
   */
  public void checkForNote() {

    photoSensor.isNoteOnBoard();

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

    String position = "Center";
    if (!blueAlliance) {
      if (autoPosition == "Left") {
        position = "Right";
      }
      else if (autoPosition == "Right") {
        position = "Left";
      }
    }
    else 
    {
      position = autoPosition;
    }

    String autoDecision = position + Integer.toString(autoNotes);
    Command autoCommand = auto1NoteCenterCommand;

    System.out.println("Auto Cmd: " + autoDecision);
    SmartDashboard.putString("Auto Cmd: ", autoDecision);
    switch (autoDecision) {

      case "Left1":
        autoCommand = auto1NoteLeftCommand;
        break;

      case "Left2":
        autoCommand = auto2NoteLeftCommand;
        break;

      case "Left3":
        autoCommand = auto2NoteLeftCommand;
        break;

      case "Right1":
        autoCommand = auto1NoteRightCommand;
        break;

      case "Right2":
        autoCommand = auto2NoteRightCommand;
        break;

      case "Right3":
        autoCommand = auto2NoteRightCommand;
        break;

      case "Center1":
        autoCommand = auto1NoteCenterCommand;
        break;

      case "Center2":
        autoCommand = auto2NoteCenterCommand;
        break;

      case "Center3":
        autoCommand = auto2NoteCenterCommand;
        break;

      default: 
        autoCommand = auto1NoteCenterCommand;
        break;
        
    }

    return autoCommand;

  }

  /*
   * Zero positions of all mechanisms and gyro
   */
  public void zeroRobot() {
    shooterAngle.zeroEncoder();
    swervedrivewpi.zeroGyro();
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
    SmartDashboard.putBoolean("Note On Board", noteOnBoard);

    //Update Ready to Shoot
    SmartDashboard.putBoolean("Ready To Shoot", readyToShoot);

    //Update Climber Position
    SmartDashboard.putBoolean("Climb Extended", ClimberExtended);

    //Update Gyro Position
    SmartDashboard.putNumber("Gyro Angle", swervedrivewpi.getGyroAngle());

    //Update Pressure
    SmartDashboard.putNumber("Pressure", pneumatic.getPressure());

    //Update drive values
    SmartDashboard.putBoolean("Slow Mode", isSlowMode);
    SmartDashboard.putBoolean("Impact Detected", impactDetected);

  }

}