package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustAndShoot;
import frc.robot.commands.FullSystemCommandsTeleop.Shoot;

public class ShootNoteInAuto extends SequentialCommandGroup{
    /** Creates a new ShootNoteInAuto */
    public ShootNoteInAuto() {
        addCommands(
            new AutoAdjustAndShoot().withTimeout(1.25));
    }
}
