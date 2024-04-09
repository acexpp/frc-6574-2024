package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootSubwoofer;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootSubwoofer;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustShooterWrist;

public class ShootSubwooferInAuto extends SequentialCommandGroup{
    /** Creates a new ShootSubwooferInAuto */
    public ShootSubwooferInAuto() {
        addCommands(
            new AdjustAndShootSubwoofer().withTimeout(0.75
            ));
    }
}