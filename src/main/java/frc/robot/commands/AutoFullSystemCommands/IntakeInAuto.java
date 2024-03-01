package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeNote;

public class IntakeInAuto extends SequentialCommandGroup{
    /** Creates a new IntakeInAuto */
    public IntakeInAuto() {
        addCommands(new IntakeNote().withTimeout(0.8));
    }
}
