// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.moveAndRotate;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.moveToOffset;
import frc.robot.commands.moveToRotation;
import frc.robot.commands.moveAndRotate;
import frc.robot.commands.moveAndRotateBlue;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ScoringMotor;

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
  // The robot's subsystems and commands
  private final CommandXboxController driver = new CommandXboxController(1); 
  //private final CommandXboxController scorer = new CommandXboxController(0); 
  private final SendableChooser<Command> autoChooser;
  private final Swerve s_Swerve = new Swerve();
  private final ScoringMotor s_Elevator = new ScoringMotor();
  private final moveAndRotate MR_tag = new moveAndRotate(s_Swerve, s_Swerve::getPose);
  private final Trigger yButton = driver.y();
  private final Trigger aButton = driver.a();
  private final Trigger bButton = driver.b();
  //private final Trigger scorerBButton = scorer.b();
  //private final Trigger scorerAButton = scorer.a();
  private final Trigger backButton = driver.back(); // Assuming `back` is already defined
  private final Trigger start = driver.start();
  private final Trigger rightTrigger = driver.rightTrigger();
  private final Trigger leftTrigger = driver.leftTrigger();
  private final Trigger povUp = driver.povUp();
  private final Trigger povDown = driver.povDown();
  private final Trigger povLeft = driver.povLeft();
  // TODO Add offsets
  private final moveToOffset moveL = new moveToOffset(s_Swerve, s_Swerve::getPose);
  private final moveToOffset moveR = new moveToOffset(s_Swerve, s_Swerve::getPose);

  /* The container for the robot. subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Pathplanner commands
    NamedCommands.registerCommand("ScoreL1",
        new InstantCommand(() -> ScoringMotor.runMotorForwards()).andThen(new WaitCommand(5)));
    
    autoChooser = AutoBuilder.buildAutoChooser();
    // System.out.println(AutoBuilder.getAllAutoNames());
    SmartDashboard.putData("AutoChooser", autoChooser);
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false,
            () -> driver.leftTrigger().getAsBoolean(),
            () -> driver.x().getAsBoolean()));

    // TODO Remove manual control later
    // s_Elevator.setDefaultCommand(
    //     s_Elevator.manualShaftControl(() -> driver.leftBumper().getAsBoolean(),
    //         () -> driver.rightBumper().getAsBoolean()));

    /*s_Climber.setDefaultCommand(
    s_Climber.manualControl(
    ()-> driver.leftBumper().getAsBoolean(),
    ()-> driver.rightBumper().getAsBoolean(),
    ()-> driver.x().getAsBoolean()
    ));*/

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
    yButton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // leftBumper.onTrue(s_Climber.arise(Constants.Climber.kClimberOut));
    // rightBumper.onTrue(s_Climber.arise(Constants.Climber.kClimberIn));

    // xButton.onTrue(new InstantCommand(() -> s_Climber.trapOn()));
    // xButton.onFalse(new InstantCommand(() -> s_Climber.trapOff()));
    yButton.whileTrue(new InstantCommand(() -> ScoringMotor.runMotorForwards()));
    yButton.onFalse(new InstantCommand(() -> ScoringMotor.stop()));
    // rightBumper.onTrue(new InstantCommand(() -> s_Spitter.OuttakeIn()));
    // bButton.onTrue(s_Elevator.setHeight());
    aButton.whileTrue(MR_tag.andThen(Commands.runOnce(() -> ScoringMotor.runMotorForwards())));//.until(()->s_Spitter.curr_state.equals(s_Spitter.states[0])));
    aButton.onFalse(Commands.runOnce(() -> ScoringMotor.stop()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(()->{
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)){
        s_Swerve.setGyro(180);
      } 
      else{ 
        s_Swerve.zeroGyro();
      }}).andThen(autoChooser.getSelected());//new PathPlannerAuto("MAuto"));
    
  }
}
 