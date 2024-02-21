package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootNote;

public class ShootNoteInAuto extends SequentialCommandGroup{
    /** Creates a new ShootInAuto */
    public ShootNoteInAuto() {
        addCommands(new ShootNote().withTimeout(2));
    }
}
