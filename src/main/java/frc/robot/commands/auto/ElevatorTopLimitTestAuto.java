package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SimpleElevator;

public class ElevatorTopLimitTestAuto extends SequentialCommandGroup {
    
    public ElevatorTopLimitTestAuto(SimpleElevator elevator) {
        addCommands(
            // First phase: Move up at 50% speed for 2 seconds
            Commands.startEnd(
                () -> elevator.setSpeed(0.5),
                () -> elevator.stop(),
                elevator
            ).withTimeout(1.0),
            
            // Second phase: Move up at 30% speed for 1 second
            Commands.startEnd(
                () -> elevator.setSpeed(0.3),
                () -> elevator.stop(),
                elevator
            ).withTimeout(1.0),
            
            // Final phase: Move up at 20% speed until top limit is reached
            new MoveUntilTopLimit(elevator, 0.2),
            
            // Wait 1 second at the top position
            Commands.waitSeconds(1.0)
            
            // // Move down at 30% speed for 3 seconds
            // Commands.startEnd(
            //     () -> elevator.setSpeed(-0.3),
            //     () -> elevator.stop(),
            //     elevator
            // ).withTimeout(3.0)
        );
    }
    
    /**
     * Custom command that moves the elevator until the top limit switch is reached
     */
    private static class MoveUntilTopLimit extends Command {
        private final SimpleElevator elevator;
        private final double speed;
        
        public MoveUntilTopLimit(SimpleElevator elevator, double speed) {
            this.elevator = elevator;
            this.speed = speed;
            addRequirements(elevator);
        }
        
        @Override
        public void execute() {
            elevator.setSpeed(speed);
        }
        
        @Override
        public boolean isFinished() {
            // Finish when the elevator reaches the top
            return elevator.isAtTop();
        }
        
        @Override
        public void end(boolean interrupted) {
            elevator.stop();
        }
    }
}