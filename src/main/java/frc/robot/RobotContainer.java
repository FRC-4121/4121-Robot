
package frc.robot;

import static frc.robot.Constants.*;

import org.opencv.core.Mat;

import frc.robot.subsystems.*;
import frc.robot.subsystems.cameras.CameraBuilder;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;



public class RobotContainer {
  
  // Camera
  CvSink camSink;
  CvSource camSource;

  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final XboxController secondaryXbox = new XboxController(1);
  private final Joystick launchpad = new Joystick(2);
  //private final Joystick testbed = new Joystick(3);
  

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
  private final ChangeSpeedCommand changeSpeedCommand = new ChangeSpeedCommand();

 //Auto Commands
 private final AutoDrive autoDriveCommand = new AutoDrive(swervedrive,0.6,120,180,0,0,20,table);
 //private final AutoBalance autoBalanceCommand = new AutoBalance(swervedrive,0.25,0,20,table);
 private final AutoGroup1 autoGroup = new AutoGroup1(swervedrive, table);
 private final AutoDriveAndBalance autoDriveAndBalance = new AutoDriveAndBalance(swervedrive,table);
 private final AutoPlaceAndBalance autoPlaceAndBalanceCommand = new AutoPlaceAndBalance(swervedrive,table,armRotate,pneumatic,arm,wrist,grabber);
 private final AutoArmStartPos autoArmStart = new AutoArmStartPos(armRotate, pneumatic,arm);
 private final AutoLoadPos autoArmLoad = new AutoLoadPos(armRotate,pneumatic,arm,wrist);
 private final AutoArmTravelPos autoArmTravel = new AutoArmTravelPos(armRotate, pneumatic, arm, wrist);
 private final AutoArmFloorPos autoArmFloor = new AutoArmFloorPos(armRotate,pneumatic,arm, wrist);
 private final AutoArmMidPos autoArmMid = new AutoArmMidPos(armRotate,pneumatic,arm, wrist, grabber);
 private final AutoArmHighPos autoArmHigh = new AutoArmHighPos(armRotate,pneumatic,arm, wrist, grabber);
 private final AutoArmHighCone autoArmHighGoal = new AutoArmHighCone(armRotate,pneumatic,arm, wrist, grabber);
 private final AutoMoveWrist autoMoveWrist = new AutoMoveWrist(wrist,0.5,10);

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
  //private final Trigger applyBrakeButton;
  //private final Trigger releaseBrakeButton;
  private final Trigger autoArmLoadButton;
  private final Trigger changeSpeedButton;
  
  //launchpad buttons/switches
  //private final JoystickButton killAutoButton;
  private final Trigger killAutoButton;
  private final Trigger autoArmTravelButton;
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
    //applyBrakeButton = new JoystickButton(xbox, xboxXButton);
    //releaseBrakeButton = new JoystickButton(xbox, xboxYButton);
    autoArmLoadButton = new JoystickButton(xbox, xboxYButton);
    changeSpeedButton = new JoystickButton(xbox, xboxXButton);
    
    //Going to use triggers for these
    rotateArmDownButton = new JoystickButton(xbox,xboxAButton);
    rotateArmUpButton = new JoystickButton(xbox,xboxBButton);  

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    autoArmTravelButton = new JoystickButton(launchpad,LaunchPadSwitch1top);
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

    //Make sure the positions are zero
    zeroRobot();

    //Make sure the grabber is closed
    pneumatic.grab();

    // Start driver cameras
    startDriverCams();

    //No feedback yet
    SmartDashboard.putBoolean("Move Right", false);
    SmartDashboard.putBoolean("Move Left", false);
    SmartDashboard.putBoolean("On Target", false);

    

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
    autoArmTravelButton.onTrue(autoArmTravel);
    autoArmFloorButton.onTrue(autoArmFloor);
    autoArmMidButton.onTrue(autoArmMid);
    autoArmHighButton.onTrue(autoArmHigh);
    autoArmLoadButton.onTrue(autoArmLoad);
    changeSpeedButton.onTrue(changeSpeedCommand);

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
    //applyBrakeButton.whileTrue(applyBrakeCommand);
    //releaseBrakeButton.whileTrue(releaseBrakeCommand);
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
  

  // Get the correct auto command
  public Command getAutonomousCommand() {
    //return autoDriveCommand;
    //return autoBalanceCommand;
    //return autoGroup;
    //return autoPlaceAndBalanceCommand;
    //return autoDriveAndBalance;
    return autoArmHighGoal;
  }


  // Start driver cameras
  public void startDriverCams() {

    // UsbCamera grabCamera = new UsbCamera("Grab Cam", 0);
    // grabCamera.setResolution(160, 120);
    // grabCamera.setBrightness(100);
    // grabCamera.setFPS(24);
    //CameraServer.startAutomaticCapture();
    // camSink = CameraServer.getVideo();
    // camSource = CameraServer.putVideo("Grab Cam", 160, 120);


    //SmartDashboard.putBoolean("Cam Started",true);

  }


  public void streamCams() {

    // camSource = CameraServer.putVideo("Grab Cam", 160, 120);
    // Mat frame = new Mat();
    // camSink.grabFrame(frame);
    // camSource.putFrame(frame);
  }

  public void zeroRobot() {
    
    //Zero the encoders when robot starts up
    armRotate.zeroEncoder();
    arm.zeroExtendEncoder();

    //Make sure that the wrist is starting at 0
    currentWristPosition = 0.0;

    //Put the encoder value on the smart dashboard
    SmartDashboard.putNumber("Rotate Position", armRotate.getMasterEncoder());
    SmartDashboard.putNumber("Extend Position", arm.getExtendEncoder());
    
    //Put the wrist position on the dashboard on startup
    SmartDashboard.putNumber("WristPosition",currentWristPosition);

  }

  public void stopPi() {

    table.putVisionDouble("RobotStop", 1.0);

  }

  public void checkTargetAlignment(){

    if (purpleButton.getAsBoolean() == true) {
      
      if(table.getVisionDouble("CubesFound") > 0 ) {
        
        double cubeOffset = table.getVisionDouble("Cubes.0.offset");
        if(cubeOffset < targetCubeOffset - visionTolerance ) {
          
          SmartDashboard.putBoolean("Move Right", true);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", false);

        } else if(cubeOffset > targetCubeOffset + visionTolerance) {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", true);
          SmartDashboard.putBoolean("On Target", false);

        } else {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", true);

        }
      }
 
    } else if (yellowButton.getAsBoolean() == true) {
      
      if(table.getVisionDouble("ConesFound") > 0 ) {
        
        double coneOffset = table.getVisionDouble("Cones.0.offset");
        if(coneOffset < targetConeOffset - visionTolerance ) {
          
          SmartDashboard.putBoolean("Move Right", true);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", false);

        } else if(coneOffset > targetConeOffset + visionTolerance) {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", true);
          SmartDashboard.putBoolean("On Target", false);

        } else {

          SmartDashboard.putBoolean("Move Right", false);
          SmartDashboard.putBoolean("Move Left", false);
          SmartDashboard.putBoolean("On Target", true);

        }
      }

    } else {
        
      SmartDashboard.putBoolean("Move Right", false);
      SmartDashboard.putBoolean("Move Left", false);
      SmartDashboard.putBoolean("On Target", false);

    }
 
  }
}