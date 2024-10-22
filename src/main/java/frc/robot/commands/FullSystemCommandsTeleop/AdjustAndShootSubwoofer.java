package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.ShootSubwoofer;

public class AdjustAndShootSubwoofer extends SequentialCommandGroup{
    public AdjustAndShootSubwoofer()
    {
        addCommands(new ParallelCommandGroup(
            // new SetShooterWristPosition(Constants.RobotConstants.shooterWristSubwoofer),
            new SetShooterWristPosition(0.510), 
            //new AutoAdjustShooterWrist(RobotContainer.limelight.getDistanceToTarget()),
            new ShootSubwoofer()));
    }
}

