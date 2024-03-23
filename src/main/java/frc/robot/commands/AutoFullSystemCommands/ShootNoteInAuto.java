package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustShooterWrist;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustWristWithIntake;
import frc.robot.commands.FullSystemCommandsTeleop.Shoot;

public class ShootNoteInAuto extends SequentialCommandGroup{
    /** Creates a new ShootInAuto */
    public ShootNoteInAuto() {
        addCommands(
            new AutoAdjustWristWithIntake().withTimeout(1),
            new Shoot().withTimeout(4)
            );
    }
}
