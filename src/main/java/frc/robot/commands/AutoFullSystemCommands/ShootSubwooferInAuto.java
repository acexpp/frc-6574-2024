package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.ShootSubwoofer;

public class ShootSubwooferInAuto extends SequentialCommandGroup{
    /** Creates a new ShootSubwooferInAuto */
    public ShootSubwooferInAuto() {
        addCommands(new ParallelCommandGroup(
            new SetShooterWristPosition(0.262),
            new ShootSubwoofer().withTimeout(1.25)));
    }
}