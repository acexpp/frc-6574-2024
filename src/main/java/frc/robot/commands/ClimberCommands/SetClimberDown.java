package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetClimberDown extends Command{
    public SetClimberDown(){
        addRequirements(RobotContainer.climber);

    }
    public void initialize() {
        RobotContainer.climber.driveClimber(.4);
    }

    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      RobotContainer.climber.driveClimber(0);
    }
}
