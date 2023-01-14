// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveSparkMax;
import frc.robot.commands.MoveSparkMaxBackward;
import frc.robot.commands.MoveSparkMaxForward;
import frc.robot.commands.moveVictor;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GroupedTalons;
import frc.robot.subsystems.VictorTest;
import frc.robot.subsystems.SparkMaxTest;

import frc.robot.commands.MoveTalonForward;
import frc.robot.commands.MoveTalonWithLeftJoystick;
import frc.robot.commands.MoveTalonsWithRightJoystick;
import frc.robot.commands.MoveVictorBackward;
import frc.robot.commands.MoveVictorForward;
import frc.robot.commands.RightTalonStop;
import frc.robot.commands.SparkMaxStop;
import frc.robot.commands.VictorStop;
import frc.robot.commands.MoveTalonBackward;
import frc.robot.subsystems.LeftTalon;
import frc.robot.subsystems.RightTalon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static final double KDeadZone = 0.05;
  
  //Joystick Axis IDs 
  private static final int KLeftYAxis = 1;
  private static final int KRightYAxis = 3;
  private static final int KLeftXAxis = 0;
  private static final int KRightXAxis = 2;

  //Logitech Button Constants
  public static final int KLogitechButtonX = 1;
  public static final int KLogitechButtonA = 2;
  public static final int KLogitechButtonB = 3;
  public static final int KLogitechButtonY = 4;
  public static final int KLogitechLeftBumper = 5; 
  public static final int KLogitechRightBumper = 6;
  public static final int KLogitechLeftTrigger = 7;
  public static final int KLogitechRightTrigger = 8;

  //Xbox Button Constants
  public static final int KXboxButtonA = 1; 
  public static final int KXboxButtonB = 2;
  public static final int KXboxButtonX = 3;  
  public static final int KXboxButtonY = 4; 
  public static final int KXboxLeftBumper = 5; 
  public static final int KXboxRightBumper = 6; 
  public static final int KXboxSelectButton = 7; 
  public static final int KXboxStartButton = 8; 
  public static final int KXboxLeftTrigger = 2; 
  public static final int KXboxRightTrigger = 3; 

  //Game Controllers
  public static Joystick logitech;
  public static XboxController xbox; 
  //Controller Buttons/Triggers
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button
  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect;
  public Trigger xboxBtnRT, xboxBtnLT;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final VictorTest victor = new VictorTest();
  private final SparkMaxTest sparkMax = new SparkMaxTest();
  private final LeftTalon leftTalon = new LeftTalon();
  private final RightTalon rightTalon = new RightTalon();
  private final GroupedTalons groupedTalons = new GroupedTalons();

  
  // commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final MoveTalonForward moveTalonForward = new MoveTalonForward(rightTalon);
  private final RightTalonStop rightTalonStop = new RightTalonStop(rightTalon);
  private final MoveTalonBackward moveTalonBackward = new MoveTalonBackward(rightTalon);

  private final moveVictor moveVictor = new moveVictor(victor);
  private final MoveVictorForward moveVictorForward = new MoveVictorForward(victor);
  private final MoveVictorBackward moveVictorBackward = new MoveVictorBackward(victor);
  private final VictorStop victorStop = new VictorStop(victor);

  private final MoveSparkMax moveSparkMax = new MoveSparkMax(sparkMax);
  private final MoveSparkMaxForward moveSparkMaxForward = new MoveSparkMaxForward(sparkMax);
  private final MoveSparkMaxBackward moveSparkMaxBackward = new MoveSparkMaxBackward(sparkMax);
  private final SparkMaxStop sparkMaxStop = new SparkMaxStop(sparkMax);

  private final MoveTalonWithLeftJoystick moveTalonWithLeftJoystick = new MoveTalonWithLeftJoystick(leftTalon);
  private final MoveTalonsWithRightJoystick moveTalonWithRightJoystick = new MoveTalonsWithRightJoystick(groupedTalons);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    groupedTalons.setDefaultCommand(moveTalonWithRightJoystick);
    leftTalon.setDefaultCommand(moveTalonWithLeftJoystick);
    rightTalon.setDefaultCommand(rightTalonStop);
    victor.setDefaultCommand(victorStop);
    sparkMax.setDefaultCommand(sparkMaxStop);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxBtnA.whenHeld(moveVictorForward);
    xboxBtnB.whenHeld(moveVictorBackward);

    logitechBtnA.whenHeld(moveSparkMaxForward);
    logitechBtnB.whenHeld(moveSparkMaxBackward);

    logitechBtnLB.whenHeld(moveTalonForward);
    logitechBtnRB.whenHeld(moveTalonBackward);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }


  public static double scaleBetween(double unscaledNum, double minAllowed, double maxAllowed, double min, double max) {
    return (maxAllowed - minAllowed) * (unscaledNum - min) / (max - min) + minAllowed;
  }
       
  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getY();
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0; 
  }

  public double getLogiRightXAxis() {
    double X = logitech.getZ();
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0; 
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getX();
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if(X > KDeadZone || X < -KDeadZone)
      return X;
    else 
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }
  public boolean joystickThreshold(double triggerValue) {
      if (Math.abs(triggerValue) < .09) 
        return false;
      else 
        return true;
    }
}
