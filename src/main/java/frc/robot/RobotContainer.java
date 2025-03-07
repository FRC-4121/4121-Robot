
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

public class RobotContainer {

  // ===Controllers===//

  // Declare Driver Controllers
  private final XboxController xbox;
  private final XboxController secondaryXbox;
  private final Joystick launchpad;

  // ===Subsystems===//

  // Declare Subsystems
  private final SwerveDriveWPI swerve;
  private final ElevatorMM elevator;
  private final CClaw claw;
  private final Climber climber;

  // ===Extra Systems===//

  // Declare Extra Systems
  private final NetworkTableQuerier table;
  private final AcousticSensor coralSensor;

  // ===COMMANDS===//

  // Declare Driving Commands
  // private final DriveWithJoysticks driveCommand = new
  // DriveWithJoysticks(swervedrive, xbox, table);
  private final DriveWithJoysticks fieldDriveCommand;
  private final ChangeSpeedCommand changeSpeedCommand;
  private final ChangeDriveMode changeModeCommand;
  private final ResetRobot resetRobot;

  // Declare mechanism commands
  private final MoveElevator moveElevatorCommand;
  private final RotateCClaw rotateClawCommand;

  // Declare KillAuto Commands
  private final KillAutoCommand killAuto;

  // ===BUTTONS===//

  // Declare Xbox Buttons and Triggers
  private final Trigger changeSpeedButton;
  private final Trigger changeModeButton;
  private final Trigger clawHomeButton;
  private final Trigger climbButton;
  private final Trigger elevatorHomeButton;
  private final Trigger elevatorCoral1Button;
  private final Trigger elevatorCoral2Button;
  private final Trigger elevatorCoral3Button;
  private final Trigger elevatorCoral4Button;
  private final Trigger coralScoreButton;
  private final Trigger coralIntakeButton;
  private final Trigger algaeIntakeButton;
  private final Trigger returnHomeButton;

  // Declare Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private final Trigger resetEncodersButton;
  private final Trigger blueTeamButton;
  private final Trigger redTeamButton;
  private final Trigger parkButton;
  private final Trigger resetRobotButton;

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
    elevator = new ElevatorMM();
    claw = new CClaw();
    climber = new Climber();

    // Initialize extra systems
    table = new NetworkTableQuerier();
    coralSensor = new AcousticSensor();

    // Initialize Driving Commands
    // private final DriveWithJoysticks driveCommand = new
    // DriveWithJoysticks(swervedrive, xbox, table);
    fieldDriveCommand = new DriveWithJoysticks(swerve, xbox, table);
    changeSpeedCommand = new ChangeSpeedCommand();
    changeModeCommand = new ChangeDriveMode();
    resetRobot = new ResetRobot(claw, elevator, swerve, climber);

    // Initialize mechanism commands
    moveElevatorCommand = new MoveElevator(elevator, secondaryXbox);
    rotateClawCommand = new RotateCClaw(claw, secondaryXbox);

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
    clawHomeButton = new JoystickButton(secondaryXbox, xboxRightBumper);
    climbButton = new Trigger(() -> xbox.getRightTriggerAxis() > triggerThreshold);
    elevatorCoral1Button = new JoystickButton(secondaryXbox, xboxAButton);
    elevatorCoral2Button = new JoystickButton(secondaryXbox, xboxBButton);
    elevatorCoral3Button = new JoystickButton(secondaryXbox, xboxXButton);
    elevatorCoral4Button = new JoystickButton(secondaryXbox, xboxYButton);
    coralScoreButton = new JoystickButton(secondaryXbox, xboxRightBumper);
    coralIntakeButton = new JoystickButton(secondaryXbox, xboxLeftBumper);
    algaeIntakeButton = new Trigger(() -> secondaryXbox.getLeftTriggerAxis() > triggerThreshold);
    elevatorHomeButton = new Trigger(() -> secondaryXbox.getRightTriggerAxis() > triggerThreshold);
    resetEncodersButton = new JoystickButton(launchpad, LaunchPadSwitch2top);
    returnHomeButton = new JoystickButton(xbox, xboxAButton);

    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad, LaunchPadButton1);
    parkButton = new JoystickButton(launchpad, LaunchPadButton3);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    resetRobotButton = new JoystickButton(launchpad, LaunchPadSwitch1top);

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
    clawHomeButton.onTrue(claw.autoRotate(CClaw.ClawPositions.Home));
    elevatorHomeButton.onTrue(elevator.positionElevator(ElevatorMM.ElevatorPositions.LOAD));
    elevatorCoral1Button.onTrue(
        Commands.either(
            elevator.positionElevator(ElevatorMM.ElevatorPositions.CORAL1),
            elevator.positionElevator(ElevatorMM.ElevatorPositions.ALGAE1),
            () -> claw.hasCoral()));
    elevatorCoral2Button.onTrue(elevator.positionElevator(ElevatorMM.ElevatorPositions.CORAL2));
    elevatorCoral3Button.onTrue(elevator.positionElevator(ElevatorMM.ElevatorPositions.CORAL3));
    elevatorCoral4Button.onTrue(
        Commands.either(
            elevator.positionElevator(ElevatorMM.ElevatorPositions.CORAL4),
            elevator.positionElevator(ElevatorMM.ElevatorPositions.ALGAE2),
            () -> claw.hasCoral()));
    coralIntakeButton.onTrue(claw.intakeCoral());
    coralScoreButton.onTrue(
      Commands.either(
        claw.scoreCoral(),
        claw.algaeDeposit(),
        () -> claw.hasCoral()));
    algaeIntakeButton.onTrue(claw.algaeIntake());
    climbButton.onTrue(climber.new Climb());
    returnHomeButton.onTrue(claw.returnHome());
    resetRobotButton.onTrue(resetRobot);
    resetEncodersButton.onTrue(Commands.runOnce(() -> {
      claw.zeroIntake();
      elevator.zeroPosition();
    }, claw, elevator));
  }

  /**
   * Set default commands for all subsystems
   */
  private void configureDefaultCommands() {

    swerve.setDefaultCommand(fieldDriveCommand);
    elevator.setDefaultCommand(moveElevatorCommand);
    claw.setDefaultCommand(rotateClawCommand);

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
      Mutables.blueAlliance = false;
    else if (blueTeamButton.getAsBoolean())
      Mutables.blueAlliance = true;
    else {
      // TODO: warn someone
      Mutables.blueAlliance = true;
    }
  }

  /**
   * Park and unpark the robot
   */
  public void getParkSelection() {
    if (parkButton.getAsBoolean() == true) {
      Mutables.isParked = true;
      SmartDashboard.putBoolean("Robot Parked", true);
      // parkCommand.execute();
    } else {
      Mutables.isParked = false;
      SmartDashboard.putBoolean("Robot Parked", false);
    }
  }

  /**
   * 
   * Zero all robot sensors
   * 
   */
  public void zeroRobot() {
    swerve.zeroGyro();
    swerve.zeroEncoders();
    elevator.zeroPosition();
    claw.zeroIntake();
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
    SmartDashboard.putNumber("Gyro Yaw", swerve.getGyroYaw());

    // Update drive values
    SmartDashboard.putBoolean("Slow Mode", Mutables.isSlowMode);
    SmartDashboard.putBoolean("Impact Detected", Mutables.impactDetected);

  }

}