package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakeSpeeds;

public class AutoAdjustWristWithIntake extends SequentialCommandGroup{
    public AutoAdjustWristWithIntake() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new ParallelCommandGroup(
            new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()),
            new SetIntakeSpeeds(0, -0.2, 0).withTimeout(.5)
        ).withTimeout(1));
    }
    
}
