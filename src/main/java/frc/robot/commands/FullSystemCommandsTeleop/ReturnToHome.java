// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.SetShooterWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnToHome extends SequentialCommandGroup {
  /** Creates a new ReturnToHome. */
  public ReturnToHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SetElevatorPosition(0),
        new SetShooterWristPosition(Constants.RobotConstants.shooterWristHome)
      ),
      new InstantCommand(() -> RobotContainer.elevator.stopMotors())
    );
  }
}
