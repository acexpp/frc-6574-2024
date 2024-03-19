package frc.robot.commands.FullSystemCommandsTeleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoAdjustShooterWrist extends Command{
  /** Creates a new AutoAdjustShooterWrist. */
  private double position;
  private double tolerance = 0.005;
  
  public AutoAdjustShooterWrist(double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    position = pos;
    addRequirements(RobotContainer.shooterW);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Beginning Auto Adjust");
    RobotContainer.shooterW.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(RobotContainer.shooterW.getAbsoluteEncoderPosition() - position) <= tolerance) {
      System.out.println(position);
      System.out.println("Auto Adjust Complete");
      System.out.println(RobotContainer.shooterW.getAbsoluteEncoderPosition());
      return true;
    }
    else {
      return false;
    }
  }
}
