// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.c_Backwards;
import frc.robot.commands.c_FastForward;
import frc.robot.commands.c_Forward;
import frc.robot.commands.c_IfSmacknaForward;
import frc.robot.commands.c_SlowForward;
import frc.robot.commands.c_FastForward;
import frc.robot.commands.c_Stop;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_driveWithJoystick;
import frc.robot.commands.c_smackago;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain driveTrain;

  // Commands
  private final c_driveWithController driveWithController;
  private final c_driveWithJoystick driveWithJoystick;

  // Controller
  private static XboxController controller = new XboxController(0);
  public static Joystick arduino = new Joystick(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Subsystems
    driveTrain = new Drivetrain();

    // Commands

    driveWithController = new c_driveWithController(driveTrain, controller);

    driveWithJoystick = new c_driveWithJoystick(driveTrain, arduino);

    // Defaults
    driveTrain.setDefaultCommand(driveWithController);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link Joystick}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // B
    // new JoystickButton(controller, XboxController.Button.kB.value).whenHeld(new
    // c_Forward(m_exampleSubsystem));
    new JoystickButton(arduino, 1).whenHeld(new c_Forward(m_exampleSubsystem));
    // A
    new JoystickButton(arduino, 2).whenHeld(new c_IfSmacknaForward(m_exampleSubsystem));

    new JoystickButton(arduino, 3).whenHeld(new c_Stop(m_exampleSubsystem), false);

    new JoystickButton(arduino, 4).whenHeld(new c_FastForward(m_exampleSubsystem), false);

    new JoystickButton(arduino, 4).whenInactive(new c_SlowForward(m_exampleSubsystem), true);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
