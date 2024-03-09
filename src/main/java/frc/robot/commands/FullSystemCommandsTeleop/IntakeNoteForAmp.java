package frc.robot.commands.FullSystemCommandsTeleop;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeWithSensor;

public class IntakeNoteForAmp extends SequentialCommandGroup{
    
    public IntakeNoteForAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                new IntakeWithSensor().withTimeout(3)
    );
    }
}
