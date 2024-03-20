package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.SetShooterWristPosition;

public class ShooterWristStage extends SequentialCommandGroup{
    public ShooterWristStage() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ParallelCommandGroup(
            new SetShooterWristPosition(0.202), //205
            new SetIntakeSpeeds(0, -0.2, 0).withTimeout(.5)
        ));
    }
}
