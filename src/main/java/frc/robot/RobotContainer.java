
package frc.robot;

import java.util.List;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.ControlConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAngleToTarget;

import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.ExtraClasses.PhotoElecSensor;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
  
  //===Controllers===//

  // Declare Driver Controllers
  private final XboxController xbox;
  private final XboxController secondaryXbox;
  private final Joystick launchpad;

  //===Subsystems===//
  
  // Declare Subsystems
  private final SwerveDriveWPI swervedrivewpi;
  private final Intake intake;
  private final Shooter shooter;
  private final ShooterAngle shooterAngle;
  private final Pneumatics pneumatic;
  private final Processor processor;

  //===Extra Systems===//

  // Declare Extra Systems
  private final NetworkTableQuerier table;
  private final LED led;
  private PhotoElecSensor photoSensor;

  //===COMMANDS===//

  // Declare Driving Commands
  //private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final FieldDriveWithJoysticks fieldDriveCommand;
  private final ChangeSpeedCommand changeSpeedCommand;
  private final ChangeDriveMode changeModeCommand;

  // Declare Auto Commands
  //private final SendableChooser<Command> autoChooser;
  private final AutoDrive autoDriveCommand;
  private final AutoPickupNote autoPickupNoteCommand;
  private final AutoShooterAmpPos autoShooterAmpPosCommand;
  private final AutoShooterPos autoShooterPosCommand;
  private final AutoShooterAngle autoShooterAngleCommand;
  private final Auto1NoteCenter auto1NoteCenterCommand;
  private final Auto1NoteLeft auto1NoteLeftCommand;
  private final Auto1NoteRight auto1NoteRightCommand;
  private final Auto2NoteCenter auto2NoteCenterCommand;
  private final Auto2NoteLeft auto2NoteLeftCommand;
  private final Auto2NoteRight auto2NoteRightCommand;

  // Declare KillAuto Commands
  private final KillAutoCommand killAutoObject; 
  private final ChangeAutoAngle changeAutoAngle;
  
  // Declare LED Commands
  private final LEDCommand ledCommand;

  // Declare Shooter Commands
  private final RunShooterAmp ampShooterCommand;
  private final RunShooterSpeaker speakerShooterCommand;
  private final RunShooterTrap trapShooterCommand;
  private final AutoShooterSpeed shooterSpeedCommand;
  private final AutoShooterAngle autoShootMediumCommand;
  private final AutoShooterAngle autoShootFarCommand;
  private final AutoShooterBottom autoShootBottomCommand;

  // Declare Shooter Angle Commands
  private final RunAngleUp angleUpCommand;
  private final RunAngleDown angleDownCommand;
  
  // Declare Intake Commands
  private final RunIntake intakeCommand;
  private final RunProcessorBack processorBackCommand;
  private final TakeInNote takeInNoteCommand;

  // Declare Climber Commands
  private final RunClimber climberCommand;

  //===BUTTONS===//

  // Declare Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;
  private final Trigger intakeButton;
  private final Trigger speakerShootButton;
  private final Trigger ampShootButton;
  //private final Trigger autoAngleShootButton;
  private final Trigger climberButton;
  private final Trigger runAngleDownButton;
  private final Trigger runAngleUpButton;
  private final Trigger manualIntakeButton;
  private final Trigger processorBackButton;
  private final Trigger shuttleButton;
  
  // Declare Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private static JoystickButton blueTeamButton;
  private static JoystickButton redTeamButton;
  private static JoystickButton parkButton;
  private static JoystickButton leftButton;
  private static JoystickButton rightButton;
  private static JoystickButton ampAngleButton;
  private static JoystickButton autoShooterPositionButton;
  private static JoystickButton changeAutoAngleButton;

  /**
   * 
   * Class Constructor
   * 
   */
  public RobotContainer() { 
  
    // Initialize driver controllers
    xbox = new XboxController(1);
    secondaryXbox = new XboxController(0);
    launchpad = new Joystick(2);
  
    // Initialize Subsystems
    swervedrivewpi = new SwerveDriveWPI();
    intake = new Intake();
    shooter = new Shooter();
    shooterAngle = new ShooterAngle();
    pneumatic = new Pneumatics();
    processor = new Processor();

    // Initialize extra systems
    table = new NetworkTableQuerier();
    led = new LED();
    photoSensor = new PhotoElecSensor();
  
    // Initialize Driving Commands
    //private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
    fieldDriveCommand = new FieldDriveWithJoysticks(swervedrivewpi,xbox,table);
    changeSpeedCommand = new ChangeSpeedCommand();
    changeModeCommand = new ChangeDriveMode();

    // Initialize Auto Commands
    autoDriveCommand = new AutoDrive(swervedrivewpi, 0.1, 110.0, 0.0, 0.0, 0.02, 10.0);
    autoPickupNoteCommand = new AutoPickupNote(swervedrivewpi,table,0, 10);
    autoShooterAmpPosCommand = new AutoShooterAmpPos(shooterAngle,shooter,processor,intake,5.0);
    autoShooterPosCommand = new AutoShooterPos(shooterAngle, table);
    autoShooterAngleCommand = new AutoShooterAngle(shooterAngle,37,5);
    auto1NoteCenterCommand = new Auto1NoteCenter(swervedrivewpi, shooter, processor, intake);
    auto1NoteLeftCommand = new Auto1NoteLeft(swervedrivewpi, shooter, processor, intake);
    auto1NoteRightCommand = new Auto1NoteRight(swervedrivewpi, shooter, processor, intake);
    auto2NoteCenterCommand = new Auto2NoteCenter(swervedrivewpi, shooter, processor, intake, shooterAngle, table);
    auto2NoteLeftCommand = new Auto2NoteLeft(swervedrivewpi, shooter, processor, shooterAngle, intake);
    auto2NoteRightCommand = new Auto2NoteRight(swervedrivewpi, shooter, processor, shooterAngle, intake);

    // Initialize KillAuto Commands
    killAutoObject = new KillAutoCommand(); 
    changeAutoAngle = new ChangeAutoAngle();
  
    // Initialize LED Command
    ledCommand = new LEDCommand(led);

    // Initialize Shooter Commands
    ampShooterCommand = new RunShooterAmp(shooter,processor,intake,0.7);
    speakerShooterCommand = new RunShooterSpeaker(shooter, processor,intake,0.7);
    trapShooterCommand = new RunShooterTrap(shooter, processor,intake,1.0);
    shooterSpeedCommand = new AutoShooterSpeed(shooter);
    autoShootMediumCommand = new AutoShooterAngle(shooterAngle, 23500, 2);
    autoShootFarCommand = new AutoShooterAngle(shooterAngle, 27500, 2);
    autoShootBottomCommand = new AutoShooterBottom(shooterAngle, 2.0);

    // Initialize Shooter Angle Commands
    angleUpCommand = new RunAngleUp(shooterAngle);
    angleDownCommand = new RunAngleDown(shooterAngle);
  
    // Initialize Intake Commands
    intakeCommand = new RunIntake(intake,processor);
    processorBackCommand = new RunProcessorBack(processor);
    takeInNoteCommand = new TakeInNote(intake,processor,10);

    // Initialize Climber Commands
    climberCommand = new RunClimber(pneumatic);

    // Register Named Commands
    NamedCommands.registerCommand("TakeInNote", takeInNoteCommand);
    NamedCommands.registerCommand("ShootNote", speakerShooterCommand);
    NamedCommands.registerCommand("AngleMedium", autoShootMediumCommand);
    NamedCommands.registerCommand("AngleFar", autoShootFarCommand);
    NamedCommands.registerCommand("AngleBottom", autoShootBottomCommand);
    NamedCommands.registerCommand("ShootAmp", ampShooterCommand);

    // Create an auto command chooser
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    intakeButton = new JoystickButton(xbox, xboxAButton);
    speakerShootButton = new JoystickButton(secondaryXbox, xboxXButton);
    ampShootButton = new JoystickButton(secondaryXbox, xboxYButton);
    climberButton = new JoystickButton(xbox, xboxBButton);
    runAngleDownButton = new JoystickButton(secondaryXbox,xboxLeftBumber);
    runAngleUpButton = new JoystickButton(secondaryXbox,xboxRightBumber);
    processorBackButton = new JoystickButton(secondaryXbox,xboxAButton);
    manualIntakeButton = new JoystickButton(xbox,xboxLeftBumber);
    //autoAngleShootButton = new JoystickButton(secondaryXbox,xboxBButton);
    parkButton = new JoystickButton(xbox,xboxRightBumber);
    shuttleButton = new JoystickButton(secondaryXbox, xboxBButton);
    
    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    rightButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    leftButton = new JoystickButton(launchpad, LaunchPadSwitch6top);
    ampAngleButton = new JoystickButton(launchpad, LaunchPadSwitch1top);
    autoShooterPositionButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    changeAutoAngleButton = new JoystickButton(launchpad, 20);
  
    // Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Make sure the positions are zero
    zeroRobot();

  }


  //===METHODS, WHERE STUFF IS CONFIGURED===///

  /**
   *
   *  Set default commands for all subsystems
   * 
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
  
  /**
   * 
   * Assign commands to button actions
   * 
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
    shuttleButton.onTrue(trapShooterCommand);
    ampShootButton.onTrue(ampShooterCommand);
    ampAngleButton.onTrue(autoShooterAmpPosCommand);
    climberButton.onTrue(climberCommand);
    runAngleDownButton.whileTrue(angleDownCommand);
    runAngleUpButton.whileTrue(angleUpCommand);
    manualIntakeButton.whileTrue(intakeCommand);

  }

  /**
   * 
   * Sets the LED color for the selected target object
   * 
   */
  public void getColorSelection()
  {
    
    if (noteOnBoard == true) {
      
      ledColor = 0.65; //Orange
    
 
    } else {
      
      ledColor = 0.93; //Default Pattern 

    }

    led.setColor(ledColor);
  }

  /**
   * 
   * Determine the alliance color based on OI
   * switch position
   * 
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

  /**
   * 
   * Get the starting position in auto
   * Set by a switch on the OI
   * 
   */
  public void getAutoPosition() 
  {
    double position = (double)SmartDashboard.getNumber("Auto Position", 1);

    if ((leftButton.getAsBoolean() == true) || position == 0) {
      autoPosition = "Left"; 
    } else if((rightButton.getAsBoolean() == true) || position == 2) {
      autoPosition = "Right";
    } else {
      autoPosition = "Center"; 
    }

  }

  /**
   * 
   * Park and unpark the robot
   * 
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

  /**
   * 
   * Gets the value of the Auto Shooter Angle switch
   * 
   */
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

  /**
   * 
   * Get the value of the Auto Align Robot switch
   * 
   */
  public void getAngleToTargetSelection()

  {
    if (changeAutoAngleButton.getAsBoolean() == false)
    {
      AutoAngleToTarget = true;
      SmartDashboard.putBoolean("Auto Positioning", true);
      //parkCommand.execute();
    } else{
      AutoAngleToTarget = false;
      SmartDashboard.putBoolean("Auto Positioning", false);
    }
  }

  /**
   * 
   * Check for the presence of a note
   * 
   */
  public void checkForNote() {

    photoSensor.isNoteOnBoard();

  }

  /**
   * 
   * Updates the speed of the shooter motors based on which
   * AprilTags are visible
   * 
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
  
  /**
   * 
   * Return the correct auto command to the scheduler
   * 
   */
  public Command getAutonomousCommand() {

    //return (Command)autoChooser.getSelected();

    
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

    //PathPlannerPath testPath = PathPlannerPath.fromPathFile("TestPath");
      //return AutoBuilder.followPath(testPath);
    return new PathPlannerAuto("Amp3NoteAmp");


    /*switch (autoDecision) {

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

    return autoCommand;*/

  }

  /**
   * 
   * Zero positions of all mechanisms and gyro
   * 
   */
  public void zeroRobot() {
    shooterAngle.zeroEncoder();
    swervedrivewpi.zeroGyro();
  }

  public void zeroDriveEncoder() {
    swervedrivewpi.zeroEncoders();
  }

  /**
   * 
   * Send a signal to stop the Pi codes
   * 
   */
  public void stopPi() {

    table.putControlDouble("RobotStop", 1.0);

  }

  /**
   * 
   * Send updates on important values to dashboard
   * Called from RobotPeriodic
   * 
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
    SmartDashboard.putNumber("Gyro Yaw", swervedrivewpi.getGyroYaw());

    //Update Pressure
    SmartDashboard.putNumber("Pressure", pneumatic.getPressure());

    //Update drive values
    SmartDashboard.putBoolean("Slow Mode", isSlowMode);
    SmartDashboard.putBoolean("Impact Detected", impactDetected);

  }

}