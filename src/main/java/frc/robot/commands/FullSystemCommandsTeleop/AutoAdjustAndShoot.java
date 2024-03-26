package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoAdjustAndShoot extends SequentialCommandGroup{
    /** Creates a new AutoAdjustAndShoot */
    public AutoAdjustAndShoot() {
        if (RobotContainer.shooterW.limelightGetShooterAngle() >= 0.260) {

        }
        else {

        }
    }
}
