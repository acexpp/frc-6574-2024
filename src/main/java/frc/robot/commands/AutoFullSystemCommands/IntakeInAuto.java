package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeNote;

public class IntakeInAuto extends SequentialCommandGroup{
    /** Creates a new IntakeInAuto */
    public IntakeInAuto() {
        addCommands(
            new RunCommand(() -> RobotContainer.shooter.setShooterSpeed(0), RobotContainer.shooter), 
            new IntakeNote().withTimeout(3));
    }
}
