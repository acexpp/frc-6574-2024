//VERY MUCH NOT TUNED PROBABLY DOESN'T WORK AT ALL USE AT YOUR OWN RISK ok thanks :)
package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class ScoreNoteTest extends SequentialCommandGroup {
  /** Creates a new ScoreConeMid. */
  public ScoreNoteTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(RobotConstants.testElevatorPosition),
      new ParallelCommandGroup(
                               new SetShooterWristPosition(RobotConstants.shooterWristTestPos))

    );
  }
}
