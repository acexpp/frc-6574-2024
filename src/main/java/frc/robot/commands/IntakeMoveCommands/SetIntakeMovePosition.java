// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeMoveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetIntakeMovePosition extends Command {
  /** Creates a new SetWristPosition. */
  private double position;
  private double tolerance = 0.030;
  
  public SetIntakeMovePosition(double position) {
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeMove);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Beginning SetIntakePosition");
    RobotContainer.intakeMove.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeMove.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.intakeMove.getAbsoluteEncoderPositionLeft() - position) <= tolerance) {
      System.out.println("SetIntakePosition Complete");
      return true;
    }
    else {
      return false;
    }
  }
}
