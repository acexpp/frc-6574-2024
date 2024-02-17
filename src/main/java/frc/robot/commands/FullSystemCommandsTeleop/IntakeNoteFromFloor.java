//VERY MUCH NOT TUNED PROBABLY DOESN'T WORK AT ALL USE AT YOUR OWN RISK ok thanks :)
package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.IntakeMoveCommands.SetIntakeMovePosition;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class IntakeNoteFromFloor extends SequentialCommandGroup {
  /** Creates a new ScoreConeMid. */
  public IntakeNoteFromFloor() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(RobotConstants.elevatorHomePosition),
      new ParallelCommandGroup(
                               new SetShooterWristPosition(RobotConstants.shooterWristTestPos),
                               new SetIntakeMovePosition(RobotConstants.intakeMoveTestPositionDown),
                               new IntakeNote())

    );
  }
}
