
package frc.robot;

import static frc.robot.Constants.*;
////import static frc.robot.Constants.ShooterConstants.*;
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
  

  //Subsystems
  private final SwerveDrive swervedrive = new SwerveDrive();

  private final NetworkTableQuerier table = new NetworkTableQuerier();


  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox);

 //private final AutoClimb autoClimbCommand = new AutoClimb(climber);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

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
  private static JoystickButton shooterControlButton;
  private static JoystickButton lowerIntakeButton;
  private static JoystickButton raiseIntakeButton;
  //private final JoystickButton autoClimbButton;

  


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
    shooterControlButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    lowerIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    raiseIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6top); 

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
  }
  
  private void configureButtonBindings() {

    //kill auto
    killAutoButton.whenPressed( killAutoObject);
    killAutoButton.whenReleased( killAutoObject);

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

  public void getShooterSelection()
  {

    if(shooterControlButton.getAsBoolean() == true)
    {
      runAutoSpeedControl = false;
    } else {
      runAutoSpeedControl = true;
    }

  }
  

  public Command getAutonomousCommand() {
    return null;
  }
}