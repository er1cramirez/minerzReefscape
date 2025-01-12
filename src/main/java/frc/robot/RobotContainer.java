// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.auto.ComplexAutoCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.telemetry.RobotTelemetry;
import frc.robot.util.ElevatorStates;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerve = new SwerveDrivetrain();
  private final Elevator elevator = new Elevator();

  // Robot telemetry
  private final RobotTelemetry telemetry = new RobotTelemetry();

  // Controllers
  private final XboxController chassisController = new XboxController(0);
  private final XboxController mechanismController = new XboxController(1);
  // Auto Commands
  private final Command complexCommand;
  public RobotContainer() {
    configureCommands();
    configureBindings();

    // Camera
    CameraServer.startAutomaticCapture();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // Auto Commands
    complexCommand = new ComplexAutoCommand(
      swerve,
      elevator
    );

    // Configure telemetry
    telemetry.addAutoCommand("Test Autp", complexCommand);
    telemetry.initDashboard();
  }

  private void configureCommands() {
    swerve.setDefaultCommand(new TeleopDrive(
      swerve,
      () -> -chassisController.getLeftY(), // Forward/Backward
      () -> -chassisController.getLeftX(), // Left/Right
      () -> chassisController.getRightX() // Rotation
    ));
    
    // Manual control
    elevator.setDefaultCommand(new TeleopElevator(
      elevator,
      () -> (mechanismController.getRightTriggerAxis() - mechanismController.getLeftTriggerAxis())
    ));
  }
  private void configureBindings() {
    // Configure button bindings

    // Field relative drive
    new JoystickButton(chassisController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(
        () -> swerve.setFieldRelative(
          !swerve.isFieldRelative()
        )
      ));

    // Swerve module steering resync
    new JoystickButton(chassisController, XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(
        () -> swerve.calibrateSteeringEncoders()
      ));

    // Preset positions
    new JoystickButton(mechanismController, XboxController.Button.kA.value)
      .onTrue(new ElevatorControlCommand(elevator, ElevatorStates.L1));

    new JoystickButton(mechanismController, XboxController.Button.kB.value)
      .onTrue(new ElevatorControlCommand(elevator, ElevatorStates.L2));

  }

  public Command getAutonomousCommand() {
    return telemetry.getSelectedAuto();
  }
}
