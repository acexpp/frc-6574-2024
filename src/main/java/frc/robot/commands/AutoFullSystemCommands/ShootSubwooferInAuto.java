package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootSubwoofer;

public class ShootSubwooferInAuto extends SequentialCommandGroup{
    /** Creates a new ShootSubwooferInAuto */
    public ShootSubwooferInAuto() {
        addCommands(
            new AdjustAndShootSubwoofer().withTimeout(0.75
            ));
    }
}