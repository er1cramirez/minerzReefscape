// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.telemetry.RobotTelemetry;
import frc.robot.util.DriveMode;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerve = new SwerveDrivetrain();
  // Robot telemetry
  private final RobotTelemetry telemetry = new RobotTelemetry();
  // private final LedStripe ledStripe = new LedStripe();

  // Controllers
  private final XboxController chassisController = new XboxController(0);
  // private final XboxController mechanismController = new XboxController(1);
  // Auto Commands
  private final Command testAuto;
  // private final Command simpleTimedAuto;
  public RobotContainer() {
    configureCommands();
    configureBindings();
    configureSmartDashboard();

    // Camera
    CameraServer.startAutomaticCapture();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // Auto Commands
    testAuto = new InstantCommand();
    // complexCommand = new InstantCommand();

    // Configure telemetry
    telemetry.addAutoCommand("Test Auto", testAuto);
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

    // Swerve module steering resync
    new JoystickButton(chassisController, XboxController.Button.kStart.value)
      .onTrue(new InstantCommand(
        () -> swerve.calibrateSteeringEncoders()
      ));

    // Precision mode toggle
    new JoystickButton(chassisController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> swerve.setDriveMode(DriveMode.PRECISION)))
        .onFalse(new InstantCommand(() -> swerve.setDriveMode(DriveMode.NORMAL)));
  }

  private void configureSmartDashboard() {
  }

  public Command getAutonomousCommand() {
    return telemetry.getSelectedAuto();
  }
}
