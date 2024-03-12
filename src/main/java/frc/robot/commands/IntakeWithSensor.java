package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;

public class IntakeWithSensor extends Command{
    //private double speed;
  /** Creates a new IntakeWithSensor().*/
  public IntakeWithSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.shooter);
    //this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.setIntakeSpeed(-0.5, -0.5, 0.08);
    RobotContainer.shooter.setShooterSpeed(-0.08);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setIntakeSpeed(0, 0, 0);
    RobotContainer.shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RobotContainer.sensor.getRange() <= RobotConstants.inRange) {
        return true;
    }
    return false;
  }
}
