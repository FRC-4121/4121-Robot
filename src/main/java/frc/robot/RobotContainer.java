
package frc.robot;

import static frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
//import frc.robot.extraClasses.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
  
  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final XboxController xboxClimber = new XboxController(1);
  private final Joystick launchpad = new Joystick(2);
  private final Joystick testbed = new Joystick(3);
  

  //Subsystems
  private final SwerveDrive swervedrive = new SwerveDrive();

  private final NetworkTableQuerier table = new NetworkTableQuerier();

  private final Arm arm = new Arm();
  private final Wrist wrist = new Wrist();
  private final OtherGrabber grabber = new OtherGrabber();

  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final ParkCommand parkCommand = new ParkCommand(swervedrive);

 //Auto Commands
 private final AutoDrive autoDriveCommand = new AutoDrive(swervedrive,0.6,168,180,0,0,20,table);
 private final AutoBalance autoBalanceCommand = new AutoBalance(swervedrive,0.25,0,20,table);
 private final AutoGroup1 autoGroup = new AutoGroup1(swervedrive, table);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

  //Arm Commands
  private final ExtendArm extendArmCommand = new ExtendArm(arm);
  private final RetractArm retractArmCommand = new RetractArm(arm);

  //Wrist Commands
  private final RunWristForward forwardWristCommand = new RunWristForward(wrist);
  private final RunWristBack backwardWristCommand = new RunWristBack(wrist);
  
  //Grabber Commands
  private final RunGrabberWheelForward grabWheelForwardCommand = new RunGrabberWheelForward(grabber);
  private final RunGrabberWheelBackward grabWheelBackwardCommand = new RunGrabberWheelBackward(grabber);
  private final Grab grab = new Grab(grabber);
  private final LetGo letGo = new LetGo(grabber);

  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  private final JoystickButton intakeButton;
  private final JoystickButton climberExtendButton;
  private final JoystickButton climberRetractButton;
  private final JoystickButton climberRotateFrontButton;
  private final JoystickButton climberRotateBackButton;
  private final JoystickButton climberStartButton;
  private final JoystickButton shooterButton;
  private final JoystickButton loaderButton;
  private final JoystickButton raiseButton;
  private final JoystickButton runShooterButton;
  private final JoystickButton toggleShooter;
  
  //launchpad buttons/switches
  private final JoystickButton killAutoButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton redButton;
  private static JoystickButton blueButton;
  private static JoystickButton armControlButton;
  private static JoystickButton lowerIntakeButton;
  private static JoystickButton raiseIntakeButton;
  private static JoystickButton parkButton;
  //private final JoystickButton autoClimbButton;

  //testbet buttons
  private final JoystickButton extendArmButton;
  private final JoystickButton retractArmButton;
  private final JoystickButton wristForwardButton;
  private final JoystickButton wristBackwardButton;
  private final JoystickButton grabberBackwardButton;
  private final JoystickButton grabberForwardButton;
  private final JoystickButton grabButton;
  private final JoystickButton letGoButton;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
    
  //colorButtons
  redButton = new JoystickButton(launchpad,LaunchPadSwitch5top);
  blueButton = new JoystickButton(launchpad,LaunchPadSwitch5bottom);

  
    //xboxButtons
    intakeButton = new JoystickButton(xbox, xboxLeftBumber); //feeds to processor
    climberExtendButton = new JoystickButton(xboxClimber, xboxRightBumber);
    climberRetractButton = new JoystickButton(xboxClimber, xboxLeftBumber);
    climberRotateFrontButton = new JoystickButton(xboxClimber, xboxYButton);
    climberRotateBackButton = new JoystickButton(xboxClimber, xboxXButton);
    climberStartButton = new JoystickButton(xboxClimber, xboxBButton);
    shooterButton = new JoystickButton(xbox, xboxRightBumber);
    runShooterButton = new JoystickButton(xbox,xboxBButton);
    loaderButton = new JoystickButton(xbox, xboxAButton);
    raiseButton = new JoystickButton(xbox, xboxYButton);
    toggleShooter = new JoystickButton (xboxClimber, xboxAButton);
    //autoClimbButton = new JoystickButton(launchpad, LaunchPadSwitch?);

  

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    AutoPos1 = new JoystickButton(launchpad,LaunchPadDial1);
    AutoPos2 = new JoystickButton(launchpad,LaunchPadDial2);
    AutoPos3 = new JoystickButton(launchpad,LaunchPadDial3);
    redButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    blueButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    armControlButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    lowerIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    raiseIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6top); 
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);

    //testbed buttons
    extendArmButton = new JoystickButton(testbed,7); 
    retractArmButton = new JoystickButton(testbed,8); 
    wristForwardButton = new JoystickButton(testbed,5);
    wristBackwardButton = new JoystickButton(testbed,6);
    grabberForwardButton = new JoystickButton(testbed,3);
    grabberBackwardButton = new JoystickButton(testbed,4);
    grabButton = new JoystickButton(testbed,1);
    letGoButton = new JoystickButton(testbed,2);

    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();


  }






  //===METHODS,WHERE STUFF IS CONFIGURED===///


  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands(){

    //Drivetrain -> drive with xbox joysticks
    swervedrive.setDefaultCommand(driveCommand);

    //Arm's default command is the extend control
    //arm.setDefaultCommand();
  }
  
  private void configureButtonBindings() {

    //kill auto
    killAutoButton.whenPressed( killAutoObject);
    killAutoButton.whenReleased( killAutoObject);

    //testbed
    extendArmButton.whileHeld(extendArmCommand);
    retractArmButton.whileHeld(retractArmCommand);
    wristForwardButton.whileHeld(forwardWristCommand);
    wristBackwardButton.whileHeld(backwardWristCommand);
    grabberForwardButton.whileHeld(grabWheelForwardCommand);
    grabberBackwardButton.whileHeld(grabWheelBackwardCommand);
    grabButton.whileHeld(grab);
    letGoButton.whileHeld(letGo);
  }

   
  //gets the color selected for the match
  public void getColorSelection()
  {
    
    if (blueButton.getAsBoolean() == true) {
      table.setColor(2);
      SmartDashboard.putString("Ball Color", "Blue");
    } else if (redButton.getAsBoolean() == true) {
      table.setColor(1);
      SmartDashboard.putString("Ball Color", "Red");
    } else {
      table.setColor(1);
      SmartDashboard.putString("Ball Color", "Red");
    }
  }

  public void getArmSelection()
  {

    if(armControlButton.getAsBoolean() == true)
    {
      runAutoArmExtend = false;
    } else {
      runAutoArmExtend = true;
    }

  }

  public void getParkSelection()
  {

    if(parkButton.getAsBoolean() == true)
    {
      isParked = true;
      SmartDashboard.putBoolean("Robot Parked", true);
      parkCommand.execute();
    } else{
      isParked = false;
      SmartDashboard.putBoolean("Robot Parked", false);
    }
  }
  

  public Command getAutonomousCommand() {
    //return autoDriveCommand;
    //return autoBalanceCommand;
    return autoGroup;
  }
}