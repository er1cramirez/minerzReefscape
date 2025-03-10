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
// import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.subsystems.SimpleElevator;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.telemetry.RobotTelemetry;
// import frc.robot.util.ElevatorStates;
import frc.robot.util.DriveMode;
import frc.robot.commands.ArmPositionCommand;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerve = new SwerveDrivetrain();
  private final SimpleElevator elevator = new SimpleElevator();
  private final CoralGrabberArm coralArm = new CoralGrabberArm();
  // private final CoralGrabber coralGrabber = new CoralGrabber(); // Add this if not already present
  
  // Robot telemetry
  private final RobotTelemetry telemetry = new RobotTelemetry();

  // Controllers
  private final XboxController chassisController = new XboxController(0);
  private final XboxController mechanismController = new XboxController(1); // For mechanism control

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

    // Coral Arm position control using the new command
    new JoystickButton(mechanismController, XboxController.Button.kA.value)
        .onTrue(new ArmPositionCommand(coralArm, CoralGrabberArm.STOWED_POSITION));

    new JoystickButton(mechanismController, XboxController.Button.kB.value)
        .onTrue(new ArmPositionCommand(coralArm, CoralGrabberArm.SCORING_POSITION));

    // Add a mid position if desired
    new JoystickButton(mechanismController, XboxController.Button.kX.value)
        .onTrue(new ArmPositionCommand(coralArm, CoralGrabberArm.MID_POSITION));

  }

  public Command getAutonomousCommand() {
    return telemetry.getSelectedAuto();
  }
}
