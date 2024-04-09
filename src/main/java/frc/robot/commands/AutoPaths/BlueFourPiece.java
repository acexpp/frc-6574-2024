// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPaths;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueFourPiece extends SequentialCommandGroup {
  private DriveSubsystem driveSubsystem;

  private Field2d field;
  /** Creates a new BlueTwoPieceCommand. */
  public BlueFourPiece(DriveSubsystem drive) {

    driveSubsystem = drive;
    addRequirements(driveSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        2,
        1.5)
        .setKinematics(DriveConstants.kDriveKinematics);
    
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
        2,
        1.5)
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

    // robot leaves start zone and moves to pick up note at podium
    Trajectory driveToFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER, new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE2.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(Units.degreesToRadians(0)))), forwardConfig);

    Trajectory driveToScoreFirstNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE2.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER.plus(new Translation2d(Units.inchesToMeters(18), 0)), new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

    Trajectory driveToSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER.plus(new Translation2d(Units.inchesToMeters(18), 0)), new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(TrajectoryConstants.NOTE1.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(0))), forwardConfig);

    Trajectory driveToScoreSecondNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE1.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER.plus(new Translation2d(Units.inchesToMeters(18), 0)), new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

    Trajectory driveToThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER.plus(new Translation2d(Units.inchesToMeters(18), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.NOTE3.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(0))), forwardConfig);

    Trajectory driveToScoreThirdNoteTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(TrajectoryConstants.NOTE3.plus(new Translation2d(Units.inchesToMeters(6), 0)), new Rotation2d(0)),
        new Pose2d(TrajectoryConstants.FRONT_CENTER_BLUE_SUBWOOFER.plus(new Translation2d(Units.inchesToMeters(18), 0)), new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);



 
     //Simulation
    //  field = new Field2d();

    //  if (RobotBase.isSimulation()) {
    //     SmartDashboard.putData(field);

    //     field.setRobotPose(driveToFirstNoteTrajectory.getInitialPose());
      
    //     field.getObject("Drive to first Trajectory").setTrajectory(driveToFirstNoteTrajectory);
    //     field.getObject("Drive to score first Trajectory").setTrajectory(driveToScoreFirstNoteTrajectory);
    //     field.getObject("Drive to second Trajectory").setTrajectory(driveToSecondNoteTrajectory);
    //     field.getObject("Drive to score second Trajectory").setTrajectory(driveToScoreSecondNoteTrajectory);
    //     field.getObject("Drive to third Trajectory").setTrajectory(driveToThirdNoteTrajectory);
    //     field.getObject("Drive to score third Trajectory").setTrajectory(driveToScoreThirdNoteTrajectory);

    //   }

    // System.out.println("TIME: " + (driveToFirstNoteTrajectory.getTotalTimeSeconds() + driveToScoreFirstNoteTrajectory.getTotalTimeSeconds() + driveToSecondNoteTrajectory.getTotalTimeSeconds() + driveToScoreSecondNoteTrajectory.getTotalTimeSeconds() + driveToThirdNoteTrajectory.getTotalTimeSeconds() + driveToScoreThirdNoteTrajectory.getTotalTimeSeconds()));


    var thetaController = new ProfiledPIDController(
        7.2, 0, 0,
        new TrapezoidProfile.Constraints(Units.degreesToRadians(540),
            Units.degreesToRadians(720)));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        // Position controllers
        new PIDController(2.5, 0, 0),
        new PIDController(2.5, 0, 0),
        thetaController);

    SwerveControllerCommand driveToFirstNoteCommand = new SwerveControllerCommand(
        driveToFirstNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreFirstNoteCommand = new SwerveControllerCommand(
        driveToScoreFirstNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToSecondNoteCommand = new SwerveControllerCommand(
        driveToSecondNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreSecondNoteCommand = new SwerveControllerCommand(
        driveToScoreSecondNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToThirdNoteCommand = new SwerveControllerCommand(
        driveToThirdNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreThirdNoteCommand = new SwerveControllerCommand(
        driveToScoreThirdNoteTrajectory,
        driveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        holonomicDriveController,
        driveSubsystem::setModuleStates,
        driveSubsystem);
        
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> driveSubsystem.resetOdometry(driveToFirstNoteTrajectory.getInitialPose())),
      driveToFirstNoteCommand,
      driveToScoreFirstNoteCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, true, false)),
      new WaitCommand(2),
      driveToSecondNoteCommand,
      driveToScoreSecondNoteCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, true, false)),
      new WaitCommand(2),
      driveToThirdNoteCommand,
      driveToScoreThirdNoteCommand,
      new InstantCommand(() -> driveSubsystem.drive(0, 0, 0, true, true))
    );
  }
}
