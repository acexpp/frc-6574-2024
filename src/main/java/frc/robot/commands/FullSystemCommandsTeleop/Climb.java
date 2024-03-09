package frc.robot.commands.FullSystemCommandsTeleop;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class Climb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristPosition(RobotConstants.shooterWristAmpPos)
    );
  }
}
