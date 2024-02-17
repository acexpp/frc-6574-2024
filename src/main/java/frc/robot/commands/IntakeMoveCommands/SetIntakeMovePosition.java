// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeMoveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetIntakeMovePosition extends Command {
  /** Creates a new SetWristPosition. */
  private double positionL;
  private double positionR;
  private double tolerance = 0.030;
  
  public SetIntakeMovePosition(double positionLeft, double positionRight) {
    this.positionL = positionLeft;
    this.positionR = positionRight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakeMove);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Intake Move", "Beginning SetIntakePosition");
    RobotContainer.intakeMove.setPosition(positionL, positionR);
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
    if (Math.abs(RobotContainer.intakeMove.getAbsoluteEncoderPositionLeft() - positionL) <= tolerance) {
      SmartDashboard.putString("Intake Move", "SetIntakePosition Complete");
      return true;
    }
    else {
      return false;
    }
  }
}
