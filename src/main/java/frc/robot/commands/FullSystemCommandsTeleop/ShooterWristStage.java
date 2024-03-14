package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetShooterWristPosition;

public class ShooterWristStage extends SequentialCommandGroup{
    public ShooterWristStage() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new SetShooterWristPosition(0.225)
        );
    }
}
