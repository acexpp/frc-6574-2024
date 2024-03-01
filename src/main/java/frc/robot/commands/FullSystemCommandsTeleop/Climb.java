package frc.robot.commands.FullSystemCommandsTeleop;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class Climb extends SequentialCommandGroup {
  /** Creates a new ScoreNoteAmp. */
  public Climb() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristPosition(RobotConstants.shooterWristAmpPos)
    );
  }
  /*
  m_driverController.rightBumper().whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> shooter.setShooterSpeed(-Constants.RobotConstants.shooterSpeed), shooter),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new RunCommand(() -> intake.setIntakeSpeed(0, 0.8), intake)
      )
    ));
    m_driverController.rightBumper().whileFalse(new ParallelCommandGroup(
      new RunCommand(() -> shooter.setShooterSpeed(0), shooter), 
      new RunCommand(() -> intake.setIntakeSpeed(0, 0), intake)
    ));
    */
}
