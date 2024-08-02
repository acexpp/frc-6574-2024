package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.ShootSubwoofer;

public class AdjustAndShootShortDistance extends SequentialCommandGroup{
    public AdjustAndShootShortDistance() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            // new SetShooterWristPosition(0.22),
            // new Shoot(),
            new AutoAdjustAndStartShooter(RobotContainer.shooterW.limelightGetShooterAngle()),
            new SetIntakeSpeeds(0, -1, 1)
        );
    }
}
