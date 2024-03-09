package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SetShooterWristPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;

public class ScoreNoteAmp extends SequentialCommandGroup {
  /** Creates a new ScoreNoteAmp. */
  public ScoreNoteAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(RobotConstants.ampElevatorPosition),
      new ParallelCommandGroup(
                               new SetShooterWristPosition(RobotConstants.shooterWristAmpPos))

    );
  }
}
