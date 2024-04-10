package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.AutoFullSystemCommands.IntakeInAuto;

public class AutoAdjustWristWithIntake extends SequentialCommandGroup{
    public AutoAdjustWristWithIntake() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()).withTimeout(1));
    }
    
}
