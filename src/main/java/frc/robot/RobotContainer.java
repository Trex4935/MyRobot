// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.c_Backwards;
import frc.robot.commands.c_Forward;
import frc.robot.commands.c_IfSmacknaForward;
import frc.robot.commands.c_SwerveJoystick;
import frc.robot.commands.c_driveWithController;
import frc.robot.commands.c_smackago;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  //private final c_driveWithController driveWithController;

  // Controller
  private static XboxController controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Subsystems
    driveTrain = new Drivetrain();

    // Commands

    //driveWithController = new c_driveWithController(driveTrain,controller);

    // Defaults
    driveTrain.setDefaultCommand(new c_SwerveJoystick(driveTrain, 
      () -> -controller.getRawAxis(Constants.leftAxisID),
      () -> controller.getRawAxis(Constants.rightAxisID),
      () -> controller.getRawAxis(2),
      () -> !controller.getRawButton(1)
    ));


    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(controller, 1).whenPressed(() -> driveTrain.zeroHeading());
    // B
    // new JoystickButton(controller, XboxController.Button.kB.value).whenHeld(new c_Forward(m_exampleSubsystem));

    // A
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new
    // c_Backwards(m_exampleSubsystem));
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new c_IfSmacknaForward(m_exampleSubsystem));

    // Y
    //new JoystickButton(controller, XboxController.Button.kY.value).whenPressed(new c_smackago(m_exampleSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.dtMaxSpeed, Constants.dtMaxAngAcceleration);

      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, 1),
        new Translation2d(1, 0),
        new Translation2d(2, -2)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig);

    PIDController xController = new PIDController(Constants.kPxController, 0, 0);
    PIDController yController = new PIDController(Constants.kPyController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPtheta, 0, 0,
      Constants.thetaConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand sCCommand = new SwerveControllerCommand(
      trajectory,
      driveTrain::getPose,
      Constants.driveKin,
      xController, yController, thetaController,
      driveTrain::setModuleStates,
      driveTrain);
    return new SequentialCommandGroup(
      new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
      sCCommand,
      new InstantCommand(() -> driveTrain.stopModules())
    );
  }
}
