
package frc.robot;

import static frc.robot.Constants.ControlConstants.*;

import frc.robot.subsystems.*;
import frc.robot.Constants.Mutables;
import frc.robot.ExtraClasses.AcousticSensor;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

  // ===Controllers===//

  // Declare Driver Controllers
  private final XboxController xbox;
  private final XboxController secondaryXbox;

  // ===Subsystems===//

  // Declare Subsystems
  private final SwerveDriveWPI swerve;

  // ===Extra Systems===//

  // Declare Extra Systems
  private final NetworkTableQuerier table;
  private final NetworkTableQuerier.BestTag bestTags;
  private final AcousticSensor coralSensor;

  // ===COMMANDS===//

  // Declare Driving Commands
  // private final DriveWithJoysticks driveCommand = new
  // DriveWithJoysticks(swervedrive, xbox, table);
  private final DriveWithJoysticks fieldDriveCommand;
  private final ChangeSpeedCommand changeSpeedCommand;
  private final ChangeDriveMode changeModeCommand;

  // Declare KillAuto Commands
  private final KillAutoCommand killAuto;

  // ===BUTTONS===//

  // Declare Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;
  private final Trigger alignLeftButton;
  private final Trigger alignRightButton;
  private final Trigger killAutoButton;

  // ===PathPlanner=== //

  // Declare PathPlanner variables
  private final SendableChooser<Command> autoChooser;

  private final long[] tagFilter = new long[] { 6, 7, 8, 9, 10, 11 };

  /**
   * 
   * Class Constructor
   * 
   */
  public RobotContainer() {

    // Initialize driver controllers
    xbox = new XboxController(1);
    secondaryXbox = new XboxController(0);

    // Initialize Subsystems
    swerve = new SwerveDriveWPI();

    // Initialize extra systems
    table = new NetworkTableQuerier();
    bestTags = table.getBestTags("pi/tags/april");
    coralSensor = new AcousticSensor();

    // Initialize Driving Commands
    // private final DriveWithJoysticks driveCommand = new
    // DriveWithJoysticks(swervedrive, xbox, table);
    fieldDriveCommand = new DriveWithJoysticks(swerve, xbox, table);
    changeSpeedCommand = new ChangeSpeedCommand();
    changeModeCommand = new ChangeDriveMode();

    // Initialize KillAuto Commands
    killAuto = new KillAutoCommand();

    // Register named commands for PathPlanner
    // registerPathPlannerCommands();
    NamedCommands.registerCommand("Stop Drive", swerve.stopDriving());

    // Create an auto command chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    killAutoButton = new JoystickButton(xbox, xboxAButton);

    // POVs only return the polar angle; thankfully we don't care too much about it
    alignLeftButton = new JoystickButton(xbox, xboxLeftBumper);
    alignRightButton = new JoystickButton(xbox, xboxRightBumper);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default subsystem commands
    configureDefaultCommands();

    bestTags.filter = tagFilter;
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
    alignLeftButton.whileTrue(CombinedCommands.autoAlignBest(swerve, new AutoAlignBase.Alignment() {
      {
        distance = 0.14;
        offset = -0.1651 - 0.12;
        rotation = 0;
      }
    }, bestTags, tagFilter));
    alignRightButton.whileTrue(CombinedCommands.autoAlignBest(swerve, new AutoAlignBase.Alignment() {
      {
        distance = 0.14;
        offset = 0.1651 - 0.12;
        rotation = 0;
      }
    }, bestTags, tagFilter));
  }

  /**
   * Set default commands for all subsystems
   */
  private void configureDefaultCommands() {

    swerve.setDefaultCommand(fieldDriveCommand);

  }

  /**
   * Register robot commands for PathPlanner use
   */
  // private void registerPathPlannerCommands() {
  //   NamedCommands.registerCommand("Elevator L4", elevator.positionElevator(ElevatorMM.ElevatorPositions.Coral4));
  //   NamedCommands.registerCommand("Shoot Coral", claw.scoreCoral());
  // }

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
    Mutables.blueAlliance = true;
  }

  /**
   * Park and unpark the robot
   */
  public void getParkSelection() {
    // if (parkButton.getAsBoolean() == true) {
    //   Mutables.isParked = true;
    //   SmartDashboard.putBoolean("Robot Parked", true);
    //   // parkCommand.execute();
    // } else {
    //   Mutables.isParked = false;
    //   SmartDashboard.putBoolean("Robot Parked", false);
    // }
  }

  /**
   * 
   * Zero all robot sensors
   * 
   */
  public void zeroRobot() {
    swerve.zeroGyro();
    swerve.zeroEncoders();
  }

  /**
   * 
   * Zero the robot gyro
   * 
   */
  public void zeroGyro() {
    swerve.zeroGyro();
  }

  /**
   * 
   * Send updates on important values to dashboard
   * Called from RobotPeriodic
   * 
   */
  public void updateRobotStatus() {

    // Update drive mode
    SmartDashboard.putBoolean("Field Oriented", Mutables.isFieldOriented);

    // Update Gyro Position
    SmartDashboard.putNumber("Gyro Angle", swerve.getGyroAngle());
    SmartDashboard.putNumber("Gyro Angle Field", swerve.getGyroAngleField());
    SmartDashboard.putNumber("Gyro Yaw", swerve.getGyroYaw());

    // Update drive values
    SmartDashboard.putBoolean("Slow Mode", Mutables.isSlowMode);
    SmartDashboard.putBoolean("Impact Detected", Mutables.impactDetected);
  }

  /**
   * 
   * Refresh our network queries
   * Called from robotPeriodic
   * 
   */
  public void updateNTQueries() {
    bestTags.refresh();
  }

  /**
   * 
   * Clear any position requests on the motor
   * 
   */
  public void clearMotorRequests() {
    
  }

  /**
   * 
   * Set the claw safety based on the override button
   * 
   */
  public void setClawSafety() {
    
  }

}