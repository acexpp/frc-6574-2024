package frc.robot.commands.AutoFullSystemCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LimelightDriveToTarget extends Command{
    private double rotation;
    private double forward;
    private boolean fieldRelative;

    public LimelightDriveToTarget() {
        addRequirements(RobotContainer.limelight);
        addRequirements(RobotContainer.m_robotDrive);
    }

    public void initialize() {
        rotation = RobotContainer.limelight.limelight_aim_proportional();
        forward = RobotContainer.limelight.limelight_range_proportional();
        fieldRelative = false;
    }

    public void execute() {
        RobotContainer.m_robotDrive.drive(forward, 
        -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
        rotation, 
        fieldRelative, 
        true);
    }

    public void end(boolean interrupted) {
        
    }
}
