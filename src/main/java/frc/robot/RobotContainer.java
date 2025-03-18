
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
  private final NetworkTableQuerier.BestTag bestTags;
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
  private final Trigger alignLeftButton;
  private final Trigger alignRightButton;

  // Declare Launchpad (OI) Buttons/Switches
  private final Trigger killAutoButton;
  private final Trigger resetEncodersButton;
  private final Trigger blueTeamButton;
  private final Trigger redTeamButton;
  private final Trigger parkButton;
  private final Trigger resetRobotButton;
  private final Trigger safetyOverrideButton;

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
    bestTags = table.getBestTags("pi/tags/april");
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
    // registerPathPlannerCommands();
    NamedCommands.registerCommand("Elevator L4", elevator.positionElevator(ElevatorMM.ElevatorPositions.Coral4));
    NamedCommands.registerCommand("Shoot Coral", claw.scoreCoral());
    NamedCommands.registerCommand("Home Elevator", CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Load, CClaw.ClawPositions.Home));
    NamedCommands.registerCommand("Intake Coral", CombinedCommands.combinedLoad(claw, elevator)
    .withDeadline(Commands.waitSeconds(0.5)));

    // Create an auto command chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Initialize Xbox Buttons
    changeSpeedButton = new JoystickButton(xbox, xboxYButton);
    changeModeButton = new JoystickButton(xbox, xboxXButton);
    clawHomeButton = new JoystickButton(xbox, xboxAButton);
    climbButton = new JoystickButton(xbox, xboxBButton);
    // climbButton = new Trigger(() -> xbox.getRightTriggerAxis() >
    // triggerThreshold);
    elevatorCoral1Button = new JoystickButton(secondaryXbox, xboxAButton);
    elevatorCoral2Button = new JoystickButton(secondaryXbox, xboxBButton);
    elevatorCoral3Button = new JoystickButton(secondaryXbox, xboxXButton);
    elevatorCoral4Button = new JoystickButton(secondaryXbox, xboxYButton);
    coralScoreButton = new JoystickButton(secondaryXbox, xboxRightBumper);
    coralIntakeButton = new JoystickButton(secondaryXbox, xboxLeftBumper);
    algaeIntakeButton = new Trigger(() -> secondaryXbox.getLeftTriggerAxis() > triggerThreshold);
    elevatorHomeButton = new Trigger(() -> secondaryXbox.getRightTriggerAxis() > triggerThreshold);

    // POVs only return the polar angle; thankfully we don't care too much about it
    alignLeftButton = new JoystickButton(xbox, xboxLeftBumper);
    alignRightButton = new JoystickButton(xbox, xboxRightBumper);

    // Initialize Launchpad (OI) Buttons/Switches
    killAutoButton = new JoystickButton(launchpad, LaunchPadButton1);
    parkButton = new JoystickButton(launchpad, LaunchPadButton3);
    blueTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    redTeamButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    resetRobotButton = new JoystickButton(launchpad, LaunchPadSwitch1top);
    resetEncodersButton = new JoystickButton(launchpad, LaunchPadSwitch2top);
    safetyOverrideButton = new JoystickButton(launchpad, LaunchPadSwitch4);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default subsystem commands
    configureDefaultCommands();

    climber.setDefaultCommand(Commands.run(() -> {
      double trig = xbox.getLeftTriggerAxis();
      if (trig > 0.1) {
        climber.runClimber(trig * 0.5);
        return;
      }
      trig = xbox.getRightTriggerAxis();
      if (trig > 0.1) {
        climber.runClimber(-trig * 0.5);
        return;
      }
      climber.runClimber(0);
    }, climber));

    bestTags.filter = new long[] { 7, 8, 9 };
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
    clawHomeButton.onTrue(claw.returnHome());
    elevatorHomeButton.onTrue(CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Load, CClaw.ClawPositions.Home));
    elevatorCoral1Button.onTrue(
        Commands.either(
            CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Coral1, CClaw.ClawPositions.L1Score),
            CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Algae1, CClaw.ClawPositions.Algae1),
            () -> claw.hasCoral()));
    elevatorCoral2Button.onTrue(CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Coral2, CClaw.ClawPositions.Home));
    elevatorCoral3Button.onTrue(CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Coral3, CClaw.ClawPositions.Home));
    elevatorCoral4Button.onTrue(
        Commands.either(
          CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Coral4, CClaw.ClawPositions.L4Score),
            CombinedCommands.moveClaw(claw, elevator, ElevatorMM.ElevatorPositions.Algae2, CClaw.ClawPositions.Algae2),
            () -> claw.hasCoral()));
    coralIntakeButton.onTrue(CombinedCommands.combinedLoad(claw, elevator)
        .withDeadline(Commands.waitSeconds(1).andThen(Commands.idle().until(coralIntakeButton))));
    coralScoreButton.onTrue(
        Commands.either(
            CombinedCommands.shootCoral(claw, elevator),
            claw.algaeDeposit(),
            () -> claw.hasCoral()));
    algaeIntakeButton.onTrue(claw.algaeIntake());
    climbButton.onTrue(climber.new Climb());
    resetRobotButton.onTrue(resetRobot);
    resetEncodersButton.whileTrue(Commands.runOnce(() -> {
      claw.killMotor();
      climber.killMotor();
      elevator.killMotor();
      claw.zeroIntake();
      climber.zeroEncoder();
      elevator.zeroPosition();
    }));
    safetyOverrideButton.onTrue(Commands.runOnce(() -> claw.setSafety(false)));
    safetyOverrideButton.onFalse(Commands.runOnce(() -> claw.setSafety(true)));
    alignLeftButton.whileTrue(new AutoAlignBest(swerve, new AutoAlignBase.Alignment() {
      {
        distance = 0.14;
        offset = -0.1651 - 0.12;
        rotation = 0;
      }
    }, bestTags, new long[] { 7, 8, 9 }));
    alignRightButton.whileTrue(new AutoAlignBest(swerve, new AutoAlignBase.Alignment() {
      {
        distance = 0.14;
        offset = 0.1651 - 0.12;
        rotation = 0;
      }
    }, bestTags, new long[] { 7, 8, 9 }));
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
    claw.killMotor();
    elevator.killMotor();
  }

  /**
   * 
   * Set the claw safety based on the override button
   * 
   */
  public void setClawSafety() {
    claw.setSafety(!safetyOverrideButton.getAsBoolean());
    claw.setRotation(CClaw.ClawPositions.Home);
  }

}