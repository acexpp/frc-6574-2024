package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeWithSensor;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

public class IntakeNoteForAmp extends SequentialCommandGroup{
    
    public IntakeNoteForAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorPosition(RobotConstants.elevatorHomePosition),
      new ParallelCommandGroup(
                               new SetShooterWristPosition(RobotConstants.shooterWristTestPos)
                               //new IntakeWithSensor()
    ));
    }
}
