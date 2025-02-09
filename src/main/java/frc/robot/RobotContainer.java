
package frc.robot;

import static frc.robot.Constants.ControlConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAngleToTarget;

import frc.robot.subsystems.*;
import frc.robot.Constants.MechanismConstants;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;

public class RobotContainer {

  // ===Controllers===//

  // Declare Driver Controllers
  private final XboxController xbox;
  private final XboxController secondaryXbox;
  private final Joystick launchpad;

  // ===Subsystems===//

  // Declare Subsystems
  private final SwerveDriveWPI swerve;

  // ===Extra Systems===//

  // Declare Extra Systems
  private final NetworkTableQuerier table;

  // ===COMMANDS===//

  // Declare Driving Commands
  // private final DriveWithJoysticks driveCommand = new
  // DriveWithJoysticks(swervedrive, xbox, table);
  private final DriveWithJoysticks fieldDriveCommand;
  private final ChangeSpeedCommand changeSpeedCommand;
  private final ChangeDriveMode changeModeCommand;

  // Declare Auto Commands
  // private final SendableChooser<Command> autoChooser;

  // Declare KillAuto Commands
  private final KillAutoCommand killAuto;

  // ===BUTTONS===//

  // Declare Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;

  // Declare Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private final JoystickButton blueTeamButton;
  private final JoystickButton redTeamButton;
  private final JoystickButton parkButton;
  private final JoystickButton leftButton;
  private final JoystickButton rightButton;
  private final JoystickButton changeAutoAngleButton;

  // ===PathPlanner=== //

  // Declare PathPlanner variables
  private final SendableChooser<Command> autoChooser;

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
    swerve = new SwerveDriveWPI();

    // Initialize extra systems
    table = new NetworkTableQuerier();
    photoSensor = new PhotoElecSensor();

    // Initialize Driving Commands
    // private final DriveWithJoysticks driveCommand = new
    // DriveWithJoysticks(swervedrive, xbox, table);
    fieldDriveCommand = new DriveWithJoysticks(swerve, xbox, table);
    changeSpeedCommand = new ChangeSpeedCommand();
    changeModeCommand = new ChangeDriveMode();

    // Initialize KillAuto Commands
    killAuto = new KillAutoCommand();

    // Register named commands for PathPlanner
    registerPathPlannerCommands();

    // Create an auto command chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    parkButton = new JoystickButton(xbox, xboxRightBumber);

    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad, LaunchPadButton1);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    rightButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    leftButton = new JoystickButton(launchpad, LaunchPadSwitch6top);
    changeAutoAngleButton = new JoystickButton(launchpad, 20);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default subsystem commands
    configureDefaultCommands();

  }

  /**
   * Assign commands to buttons
   */
  private void configureButtonBindings() {

    // Auto Commands
    killAutoButton.onTrue(killAuto);
    killAutoButton.onFalse(killAuto);

    // Teleop Commands
    changeSpeedButton.onTrue(changeSpeedCommand);
    changeModeButton.onTrue(changeModeCommand);

  }

  /**
   * Set default commands for all subsystems
   */
  private void configureDefaultCommands() {

    // Swerve drive default command
    swerve.setDefaultCommand(fieldDriveCommand);

  }

  /**
   * Register robot commands for PathPlanner use
   */
  private void registerPathPlannerCommands() {

  }

  /**
   * 
   * Return the correct auto command to the scheduler
   * 
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Determine the alliance color based on OI
   * switch position
   */
  public void getAllianceColor() {

    if (redTeamButton.getAsBoolean())
      Constants.blueAlliance = false;
    else if (blueTeamButton.getAsBoolean())
      Constants.blueAlliance = true;
    else {
      // TODO: warn someone
      Constants.blueAlliance = true;
    }
  }

  /**
   * Park and unpark the robot
   */
  public void getParkSelection() {
    if (parkButton.getAsBoolean() == true) {
      Constants.isParked = true;
      SmartDashboard.putBoolean("Robot Parked", true);
      // parkCommand.execute();
    } else {
      Constants.isParked = false;
      SmartDashboard.putBoolean("Robot Parked", false);
    }
  }

  /**
   * 
   * Get the value of the Auto Align Robot switch
   * 
   */
  public void getAngleToTargetSelection()

  {
    if (changeAutoAngleButton.getAsBoolean() == false) {
      AutoAngleToTarget = true;
      SmartDashboard.putBoolean("Auto Positioning", true);
      // parkCommand.execute();
    } else {
      AutoAngleToTarget = false;
      SmartDashboard.putBoolean("Auto Positioning", false);
    }
  }

  /**
   * 
   * Zero the gyro position
   * 
   */
  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void zeroDriveEncoder() {
    swerve.zeroEncoders();
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

    // Update drive mode
    SmartDashboard.putBoolean("Field Oriented", Constants.isFieldOriented);

    // Update shooter position
    SmartDashboard.putBoolean("Can Shoot", Constants.readyToShoot);

    // Update Photo Sensor
    SmartDashboard.putBoolean("Note On Board", Constants.noteOnBoard);

    // Update Gyro Position
    SmartDashboard.putNumber("Gyro Angle", swerve.getGyroAngle());
    SmartDashboard.putNumber("Gyro Yaw", swerve.getGyroYaw());

    // Update drive values
    SmartDashboard.putBoolean("Slow Mode", Constants.isSlowMode);
    SmartDashboard.putBoolean("Impact Detected", Constants.impactDetected);

  }

}