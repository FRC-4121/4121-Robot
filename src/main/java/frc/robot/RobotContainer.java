
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
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
  
  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final XboxController secondaryXbox = new XboxController(1);
  private final Joystick launchpad = new Joystick(2);
  private final Joystick testbed = new Joystick(3);
  

  //Subsystems
  private final SwerveDrive swervedrive = new SwerveDrive();

  private final NetworkTableQuerier table = new NetworkTableQuerier();

  private final ArmExtend arm = new ArmExtend();
  private final ArmRotate armRotate = new ArmRotate();
  private final Wrist wrist = new Wrist();
  private final Grabber grabber = new Grabber();
  private final Pneumatics pneumatic = new Pneumatics();

  private final LED led = new LED();

  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(swervedrive, xbox, table);
  private final ParkCommand parkCommand = new ParkCommand(swervedrive);

 //Auto Commands
 private final AutoDrive autoDriveCommand = new AutoDrive(swervedrive,0.6,168,180,0,0,20,table);
 //private final AutoBalance autoBalanceCommand = new AutoBalance(swervedrive,0.25,0,20,table);
 private final AutoGroup1 autoGroup = new AutoGroup1(swervedrive, table);
 private final AutoPlaceAndBalance autoPlaceAndBalanceCommand = new AutoPlaceAndBalance(swervedrive,table);
 private final AutoArmStartPos autoArmStart = new AutoArmStartPos(armRotate, pneumatic,arm);
 private final AutoArmFloorPos autoArmFloor = new AutoArmFloorPos(armRotate,pneumatic,arm);
 private final AutoArmMidPos autoArmMid = new AutoArmMidPos(armRotate,pneumatic,arm);
 private final AutoArmHighPos autoArmHigh = new AutoArmHighPos(armRotate,pneumatic,arm);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

  //Arm Commands
  private final ExtendArm extendArmCommand = new ExtendArm(arm);
  private final RetractArm retractArmCommand = new RetractArm(arm);
  private final RotateArmUp rotateArmUpCommand = new RotateArmUp(armRotate, pneumatic);
  private final RotateArmDown rotateArmDownCommand = new RotateArmDown(armRotate, pneumatic);

  //Wrist Commands
  private final RunWristUp wristUpCommand = new RunWristUp(wrist);
  private final RunWristDown wristDownCommand = new RunWristDown(wrist);
  
  //Grabber Commands
  private final RunGrabberWheelForward grabWheelForwardCommand = new RunGrabberWheelForward(grabber);
  private final RunGrabberWheelBackward grabWheelBackwardCommand = new RunGrabberWheelBackward(grabber);
  private final Grab grab = new Grab(pneumatic);
  private final LetGo letGo = new LetGo(pneumatic);

  //LED Command
  private final LEDCommand ledCommand = new LEDCommand(led);

  //Brake Commands
  private final ApplyBrake applyBrakeCommand = new ApplyBrake(pneumatic);
  private final ReleaseBrake releaseBrakeCommand = new ReleaseBrake(pneumatic);

  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  // private final JoystickButton extendArmButton;
  // private final JoystickButton retractArmButton;
  // private final JoystickButton rotateArmForwardButton;
  // private final JoystickButton rotateArmBackwardButton;
  // private final JoystickButton wristForwardButton;
  // private final JoystickButton wristBackwardButton;
  // private final JoystickButton grabberBackwardButton;
  // private final JoystickButton grabberForwardButton;
  // private final JoystickButton grabButton;
  // private final JoystickButton letGoButton;

  private final Trigger extendArmButton;
  private final Trigger retractArmButton;
  private final Trigger rotateArmUpButton;
  private final Trigger rotateArmDownButton;
  private final Trigger wristUpButton;
  private final Trigger wristDownButton;
  private final Trigger grabberBackwardButton;
  private final Trigger grabberForwardButton;
  private final Trigger grabButton;
  private final Trigger letGoButton;
  private final Trigger applyBrakeButton;
  private final Trigger releaseBrakeButton;
  
  //launchpad buttons/switches
  //private final JoystickButton killAutoButton;
  private final Trigger killAutoButton;
  private final Trigger autoArmStartButton;
  private final Trigger autoArmFloorButton;
  private final Trigger autoArmMidButton;
  private final Trigger autoArmHighButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton yellowButton;
  private static JoystickButton purpleButton;
  private static JoystickButton armControlButton;
  private static JoystickButton parkButton;

  //===CONSTRUCTOR===//
  public RobotContainer() { 
    
  //colorButtons
  yellowButton = new JoystickButton(launchpad,LaunchPadSwitch5top);
  purpleButton = new JoystickButton(launchpad,LaunchPadSwitch5bottom);

  
    //xboxButtons
    extendArmButton = new JoystickButton(xbox,xboxRightBumber); 
    retractArmButton = new JoystickButton(xbox,xboxLeftBumber); 
    wristUpButton = new JoystickButton(secondaryXbox,xboxAButton);
    wristDownButton = new JoystickButton(secondaryXbox,xboxBButton);
    grabberForwardButton = new JoystickButton(secondaryXbox,xboxXButton);
    grabberBackwardButton = new JoystickButton(secondaryXbox,xboxYButton);
    grabButton = new JoystickButton(secondaryXbox,xboxRightBumber);
    letGoButton = new JoystickButton(secondaryXbox,xboxLeftBumber);
    applyBrakeButton = new JoystickButton(xbox, xboxXButton);
    releaseBrakeButton = new JoystickButton(xbox, xboxYButton);
    
    //Going to use triggers for these
    rotateArmDownButton = new JoystickButton(xbox,xboxAButton);
    rotateArmUpButton = new JoystickButton(xbox,xboxBButton);  

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    autoArmStartButton = new JoystickButton(launchpad,LaunchPadSwitch1top);
    autoArmFloorButton = new JoystickButton(launchpad,LaunchPadSwitch1bottom);
    autoArmMidButton = new JoystickButton(launchpad,LaunchPadSwitch2bottom);
    autoArmHighButton = new JoystickButton(launchpad,LaunchPadSwitch2top);
    AutoPos1 = new JoystickButton(launchpad,LaunchPadDial1);
    AutoPos2 = new JoystickButton(launchpad,LaunchPadDial2);
    AutoPos3 = new JoystickButton(launchpad,LaunchPadDial3);
    yellowButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    purpleButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    armControlButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    parkButton = new JoystickButton(launchpad,LaunchPadSwitch3);

    //testbed buttons
    // extendArmButton = new JoystickButton(testbed,9); 
    // retractArmButton = new JoystickButton(testbed,10); 
    // wristForwardButton = new JoystickButton(testbed,5);
    // wristBackwardButton = new JoystickButton(testbed,6);
    // grabberForwardButton = new JoystickButton(testbed,3);
    // grabberBackwardButton = new JoystickButton(testbed,4);
    // grabButton = new JoystickButton(testbed,7);
    // letGoButton = new JoystickButton(testbed,8);
    // rotateArmBackwardButton = new JoystickButton(testbed,1);
    // rotateArmForwardButton = new JoystickButton(testbed,2);

    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    //Zero the encoders when robot starts up
    armRotate.zeroEncoder();
    arm.zeroExtendEncoder();

    //Put the encoder value on the smart dashboard
    SmartDashboard.putNumber("Rotate Position", armRotate.getMasterEncoder());
    SmartDashboard.putNumber("Extend Position", arm.getExtendEncoder());

  }






  //===METHODS,WHERE STUFF IS CONFIGURED===///


  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands() {

    //Drivetrain -> drive with xbox joysticks
    swervedrive.setDefaultCommand(driveCommand);

    //LED default command
    led.setDefaultCommand(ledCommand);

    //Arm's default command is the extend control
    //arm.setDefaultCommand();
  }
  
  private void configureButtonBindings() {

    //Auto
    killAutoButton.onTrue(killAutoObject);
    killAutoButton.onFalse(killAutoObject);
    autoArmStartButton.onTrue(autoArmStart);
    autoArmFloorButton.onTrue(autoArmFloor);
    autoArmMidButton.onTrue(autoArmMid);
    autoArmHighButton.onTrue(autoArmHigh);

    //teleop Commands
    extendArmButton.whileTrue(extendArmCommand);
    retractArmButton.whileTrue(retractArmCommand);
    rotateArmDownButton.whileTrue(rotateArmDownCommand);
    rotateArmUpButton.whileTrue(rotateArmUpCommand);
    wristUpButton.whileTrue(wristUpCommand);
    wristDownButton.whileTrue(wristDownCommand);
    grabberForwardButton.whileTrue(grabWheelForwardCommand);
    grabberBackwardButton.whileTrue(grabWheelBackwardCommand);
    grabButton.whileTrue(grab);
    letGoButton.whileTrue(letGo);
    applyBrakeButton.whileTrue(applyBrakeCommand);
    releaseBrakeButton.whileTrue(releaseBrakeCommand);
  }

   
  //gets the color selected for the match
  public void getColorSelection()
  {
    
    if (purpleButton.getAsBoolean() == true) {
      
      ledColor = 0.91; //Purple
 
    } else if (yellowButton.getAsBoolean() == true) {
      
      ledColor = 0.69; //Yellow

    } else {
      
      ledColor = 0.55; //Orange

    }
  }

  public void getArmSelection()
  {

    if (armControlButton.getAsBoolean() == true)
    {
      runAutoArmExtend = false;
    } else {
      runAutoArmExtend = true;
    }

  }

  public void getParkSelection()
  {

    if (parkButton.getAsBoolean() == true)
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
    //return autoGroup;
    return autoPlaceAndBalanceCommand;
  }
}