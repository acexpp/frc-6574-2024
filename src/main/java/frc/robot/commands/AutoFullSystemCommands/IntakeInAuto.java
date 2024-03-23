package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeAmpNoSensor;
import frc.robot.commands.IntakeNote;

public class IntakeInAuto extends SequentialCommandGroup{
    /** Creates a new IntakeInAuto */
    public IntakeInAuto() {
        addCommands(
                new IntakeNote().withTimeout(3));
        /*
        if (RobotContainer.sensor.getRange() != -1) {
            addCommands(
                new IntakeNote().withTimeout(3));
        }
        else {
            addCommands(
                new IntakeAmpNoSensor().withTimeout(3));
        }
        */
    }
}
