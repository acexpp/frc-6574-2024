package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootSubwoofer;

public class AutoAdjustAndShoot extends SequentialCommandGroup{
    /** Creates a new AutoAdjustAndShoot */
    public AutoAdjustAndShoot() {
        if (RobotContainer.shooterW.limelightGetShooterAngle() >= 0.258) {
            addCommands(new ParallelCommandGroup(
                new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()),
                new ShootSubwoofer()
            ));
        }
        else {
            addCommands(new ParallelCommandGroup(
                new AutoAdjustShooterWrist(RobotContainer.shooterW.limelightGetShooterAngle()),
                new Shoot()
            ));
        }
    }
}
