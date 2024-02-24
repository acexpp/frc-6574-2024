package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetElevatorUp extends Command{
    public SetElevatorUp(){
        addRequirements(RobotContainer.elevator);

    }
    public void initialize() {
        RobotContainer.elevator.driveElevator(0.2);
    }

    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.elevator.driveElevator(0);
    }
}
