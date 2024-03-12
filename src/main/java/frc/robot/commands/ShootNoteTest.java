package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;

public class ShootNoteTest extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNoteTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand();
    addCommands(
        new RunCommand(() -> RobotContainer.shooter.setShooterSpeed(-RobotConstants.shooterSpeed), RobotContainer.shooter),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new RunCommand(() -> RobotContainer.intake.setIntakeSpeed(0, 0, 0.7), RobotContainer.intake)
        )
    );
  }
  /* 
  Referencing:
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
