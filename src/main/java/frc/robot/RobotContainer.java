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
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.auto.ElevatorTopLimitTestAuto;
import frc.robot.subsystems.SimpleElevator;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.telemetry.RobotTelemetry;
// import frc.robot.util.ElevatorStates;
import frc.robot.util.DriveMode;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerve = new SwerveDrivetrain();
  private final SimpleElevator elevator = new SimpleElevator();
  // private final Elevator elevator = new Elevator();
  // Robot telemetry
  private final RobotTelemetry telemetry = new RobotTelemetry();

  // Controllers
  private final XboxController chassisController = new XboxController(0);
  public RobotContainer() {
    configureCommands();
    configureBindings();

    // Camera
    CameraServer.startAutomaticCapture();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // Auto Commands

    // Configure telemetry
    telemetry.addAutoCommand("Test Auto", new InstantCommand());
    // In your RobotContainer constructor or initialization method
    telemetry.addAutoCommand("Elevator Top Limit Test", 
        new ElevatorTopLimitTestAuto(elevator));
    telemetry.initDashboard();
  }

  private void configureCommands() {
    swerve.setDefaultCommand(new TeleopDrive(
      swerve,
      () -> -chassisController.getLeftY(), // Forward/Backward
      () -> -chassisController.getLeftX(), // Left/Right
      () -> chassisController.getRightX() // Rotation
    ));
  }
    
  private void configureBindings() {
    // Field relative drive
    new JoystickButton(chassisController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(
        () -> swerve.setFieldRelative(
          !swerve.isFieldRelative()
        )
      ));


    // Precision mode toggle
    new JoystickButton(chassisController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> swerve.setDriveMode(DriveMode.PRECISION)))
        .onFalse(new InstantCommand(() -> swerve.setDriveMode(DriveMode.NORMAL)));


  }

  public Command getAutonomousCommand() {
    return telemetry.getSelectedAuto();
  }
}
