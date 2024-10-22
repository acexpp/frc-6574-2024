package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetShooterWristPosition;

public class AdjustSubwoofer extends SequentialCommandGroup{
    public AdjustSubwoofer() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetShooterWristPosition(Constants.RobotConstants.shooterWristSubwoofer).withTimeout(1.5)
        );
    }
}
