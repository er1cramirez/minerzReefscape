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
import frc.robot.commands.TeleopAlgaeArm;
// import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.TeleopAlgaeGrabber;
import frc.robot.commands.TeleopClimber;
import frc.robot.commands.TeleopCoralArm;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopSimpleElevator;
import frc.robot.commands.auto.TimedAutoExample;
// import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopCoralGrabber;
// import frc.robot.commands.auto.ComplexAutoCommand;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.AlgaeGrabberArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralGrabber;
import frc.robot.subsystems.CoralGrabberArm;
import frc.robot.subsystems.LedStripe;
import frc.robot.subsystems.LedStripe.LedState;
import frc.robot.subsystems.SimpleElevator;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.telemetry.RobotTelemetry;
// import frc.robot.util.ElevatorStates;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrivetrain swerve = new SwerveDrivetrain();
  // private final Elevator elevator = new Elevator();
  private final SimpleElevator elevator = new SimpleElevator();
  private final CoralGrabber coralGrabber = new CoralGrabber();
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
  private final CoralGrabberArm coralArm = new CoralGrabberArm();
  private final AlgaeGrabberArm algaeArm = new AlgaeGrabberArm();
  private final Climber climber = new Climber();
  // Robot telemetry
  private final RobotTelemetry telemetry = new RobotTelemetry();
  private final LedStripe ledStripe = new LedStripe();

  // Controllers
  private final XboxController chassisController = new XboxController(0);
  private final XboxController mechanismController = new XboxController(1);
  // Auto Commands
  private final Command complexCommand;
  private final Command simpleTimedAuto;
  public RobotContainer() {
    configureCommands();
    configureBindings();

    // Camera
    CameraServer.startAutomaticCapture();
    SmartDashboard.putData(CommandScheduler.getInstance());
    // Auto Commands
    // complexCommand = new ComplexAutoCommand(
    //   swerve,
    //   elevator
    // );
    simpleTimedAuto = new TimedAutoExample(
        swerve,
        elevator,
        coralGrabber,
        coralArm,
        algaeGrabber
    );
    complexCommand = new InstantCommand();

    // Configure telemetry
    telemetry.addAutoCommand("Test Auto", complexCommand);
    telemetry.addAutoCommand("Simple Timed Auto", simpleTimedAuto);
    telemetry.initDashboard();
  }

  private void configureCommands() {
    swerve.setDefaultCommand(new TeleopDrive(
      swerve,
      () -> chassisController.getLeftY(), // Forward/Backward
      () -> chassisController.getLeftX(), // Left/Right
      () -> chassisController.getRightX() // Rotation
    ));
    // Manual control
    coralArm.setDefaultCommand(new TeleopCoralArm(
      coralArm,
      () -> mechanismController.getRightY()
    ));
    // Algae Arm manual control
    algaeArm.setDefaultCommand(new TeleopAlgaeArm(
      algaeArm,
      () -> mechanismController.getLeftY()
    ));
    // Manual control(elevator simplified)
    elevator.setDefaultCommand(new TeleopSimpleElevator(
      elevator,
      () -> (mechanismController.getRightTriggerAxis() - mechanismController.getLeftTriggerAxis())
    ));
    // Manual control
    // elevator.setDefaultCommand(new TeleopElevator(
    //   elevator,
    //   () -> (mechanismController.getRightTriggerAxis() - mechanismController.getLeftTriggerAxis())
    // ));
    
    climber.setDefaultCommand(new TeleopClimber(
        climber,
        () -> (chassisController.getRightTriggerAxis() - chassisController.getLeftTriggerAxis())  // Or preferred control input
    ));
  }
  private void configureBindings() {
    // Coral Grabber controls
    new JoystickButton(mechanismController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new TeleopCoralGrabber(coralGrabber, true))  // Grab while held
        .onTrue(new InstantCommand(() -> ledStripe.setState(LedState.COLLECTING)))
        .onFalse(new InstantCommand(() -> ledStripe.setState(LedState.ENABLED)));
        
    new JoystickButton(mechanismController, XboxController.Button.kRightBumper.value)
        .whileTrue(new TeleopCoralGrabber(coralGrabber, false))  // Release while held
        .onTrue(new InstantCommand(() -> ledStripe.setState(LedState.SCORING)))
        .onFalse(new InstantCommand(() -> ledStripe.setState(LedState.ENABLED)));
    // Algae Grabber controls
    new JoystickButton(mechanismController, XboxController.Button.kA.value)
        .whileTrue(new TeleopAlgaeGrabber(algaeGrabber, true));  // Grab while held

    new JoystickButton(mechanismController, XboxController.Button.kB.value)
        .whileTrue(new TeleopAlgaeGrabber(algaeGrabber, false));  // Release while held

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
    // new JoystickButton(mechanismController, XboxController.Button.kA.value)
    //   .onTrue(new ElevatorControlCommand(elevator, ElevatorStates.L1));

    // new JoystickButton(mechanismController, XboxController.Button.kB.value)
    //   .onTrue(new ElevatorControlCommand(elevator, ElevatorStates.L2));


  }

  public Command getAutonomousCommand() {
    return telemetry.getSelectedAuto();
  }
}
