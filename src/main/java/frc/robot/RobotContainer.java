
package frc.robot;

import static frc.robot.Constants.*;

import org.opencv.core.Mat;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
//import frc.robot.extraClasses.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;



public class RobotContainer {

  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final Joystick launchpad = new Joystick(2);
  //private final Joystick testbed = new Joystick(3);
  

  //Subsystems
  private final Drivetrain tankDrive = new Drivetrain();
  private final Shooter ringShooter = new Shooter();


  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(tankDrive, xbox);

  // Shooter commands
  private final ShootNote shootCommand = new ShootNote(ringShooter);
  private final IntakeNote intakeCommand = new IntakeNote(ringShooter);


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

  private final JoystickButton shootNote;
  private final JoystickButton intakeNote;

  //===CONSTRUCTOR===//
  public RobotContainer() { 

    shootNote = new JoystickButton(xbox, xboxRightBumber);
    intakeNote = new JoystickButton(xbox, xboxLeftBumber);

    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

  }


  //===METHODS,WHERE STUFF IS CONFIGURED===///

  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands() {

    tankDrive.setDefaultCommand(driveCommand);

  }
  
  private void configureButtonBindings() {

    //teleop Commands
    shootNote.whileTrue(shootCommand);
    intakeNote.whileTrue(intakeCommand);
  }

  /*
   * Return the correct auto command to the scheduler
   */
  public Command getAutonomousCommand() {

    return driveCommand;

  }


}