package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
// import frc.robot.telemetry.SwerveTelemetry;
import frc.robot.util.ElevatorStates;

public class ComplexAutoCommand extends SequentialCommandGroup {
    public ComplexAutoCommand(
            SwerveDrivetrain swerve,
            Elevator elevator) {  // Remove swerveTelemetry parameter
            
        Trajectory trajectory1 = createTrajectory(
            swerve,
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(2, 4, new Rotation2d())
        );

        Trajectory trajectory2 = createTrajectory(
            swerve,
            new Pose2d(2, 0, new Rotation2d()),
            new Pose2d(0, 2, new Rotation2d())
        );
        
        addCommands(
            // Move to scoring position
            new SwerveTrajectoryCommand(
                "Score Approach",
                swerve,
                trajectory1,
                Rotation2d.fromRadians(Math.PI)
            ),
            
            // Score game piece
            new ElevatorControlCommand(elevator, ElevatorStates.L3),
            new WaitCommand(0.5),
            // new ScoreGamePieceCommand(),
            
            // Move to next position
            new ParallelCommandGroup(
                new SwerveTrajectoryCommand(
                    "Return Path",
                    swerve,
                    trajectory2,
                    new Rotation2d()
                ),
                new ElevatorControlCommand(elevator, ElevatorStates.L1)
            )
        );
    }
    
    private Trajectory createTrajectory(SwerveDrivetrain swerve, Pose2d start, Pose2d end) {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.MAX_SPEED,
            AutoConstants.MAX_ACCELERATION
        ).setKinematics(swerve.getKinematics());
        
        return TrajectoryGenerator.generateTrajectory(
            start,
            List.of(),
            end,
            config
        );
    }
}