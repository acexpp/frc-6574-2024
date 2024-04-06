package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.ShootSubwoofer;

public class AdjustAndShootSubwoofer extends SequentialCommandGroup{
    public AdjustAndShootSubwoofer()
    {
        addCommands(new ParallelCommandGroup(
            new SetShooterWristPosition(0.238),
            new ShootSubwoofer()));
    }
}
