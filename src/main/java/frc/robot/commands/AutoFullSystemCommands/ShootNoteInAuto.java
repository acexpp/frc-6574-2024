package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustShooterWrist;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustWristWithIntake;
import frc.robot.commands.FullSystemCommandsTeleop.Shoot;

public class ShootNoteInAuto extends SequentialCommandGroup{
    /** Creates a new ShootNoteInAuto */
    public ShootNoteInAuto() {
        addCommands(
            new Shoot().withTimeout(0.5));
    }
}
