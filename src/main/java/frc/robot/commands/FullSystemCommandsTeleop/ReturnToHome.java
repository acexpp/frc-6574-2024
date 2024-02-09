// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//VERY MUCH NOT TUNED PROBABLY DOESN'T WORK AT ALL USE AT YOUR OWN RISK ok thanks :)
package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.SetIntakeSpeedInstant;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnToHome extends SequentialCommandGroup {
  /** Creates a new ReturnWAEHome. */
  public ReturnToHome() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeSpeedInstant(0),
      new ParallelCommandGroup(
        new SetElevatorPosition(0),
        new SetShooterWristPosition(RobotConstants.shooterWristHome)
      ),
      new InstantCommand(() -> RobotContainer.intakeMove.stop()),
      new InstantCommand(() -> RobotContainer.elevator.stopMotors()),
      new InstantCommand(() -> RobotContainer.shooterW.stop())
    );
  }
}
