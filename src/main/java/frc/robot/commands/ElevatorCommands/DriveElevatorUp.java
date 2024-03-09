package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveElevatorUp extends Command{
    /** Creates a new DriveElevatorUp */
    public DriveElevatorUp(){
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.elevator);

    }
    
    // Called when the command is initially scheduled.
    public void initialize() {
        RobotContainer.elevator.driveElevator(0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.elevator.driveElevator(0);
    }
}
