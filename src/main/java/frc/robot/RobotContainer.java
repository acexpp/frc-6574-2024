// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.SetIntakeSpeeds;
import frc.robot.commands.AutoFullSystemCommands.LimelightDriveToTarget;
import frc.robot.commands.AutoFullSystemCommands.ReturnHomeAndIntakeInAuto;
import frc.robot.commands.AutoFullSystemCommands.ShootNoteInAuto;
import frc.robot.commands.AutoFullSystemCommands.ShootSubwooferInAuto;
import frc.robot.commands.AutoFullSystemCommands.StopShooter;
import frc.robot.commands.ClimberCommands.SetClimberDown;
import frc.robot.commands.ClimberCommands.SetClimberUp;
import frc.robot.commands.ElevatorCommands.DriveElevatorDown;
import frc.robot.commands.ElevatorCommands.DriveElevatorUp;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootShortDistance;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustAndShootSubwoofer;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustSubwoofer;
import frc.robot.commands.FullSystemCommandsTeleop.AdjustWristAndFeed;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustAndShoot;
import frc.robot.commands.FullSystemCommandsTeleop.AutoAdjustWristWithIntake;
import frc.robot.commands.FullSystemCommandsTeleop.PrintPose;
import frc.robot.commands.FullSystemCommandsTeleop.ReturnHomeAndIntake;
import frc.robot.commands.FullSystemCommandsTeleop.ReturnToHome;
import frc.robot.commands.FullSystemCommandsTeleop.ScoreNoteAmp;
import frc.robot.commands.FullSystemCommandsTeleop.Shoot;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.armtest.ArmSubsystem;
import frc.robot.subsystems.armtest.io.ArmSimIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimSubsystem;
import frc.robot.subsystems.elevator.io.ElevatorSimIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final MechanismSimulator sim;
  private final ArmSubsystem arm;
  private final ElevatorSimSubsystem elevatorSim;

  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static VisionSubsystem limelight = new VisionSubsystem(m_robotDrive);
  public static Elevator elevator = new Elevator();
  public static Shooter shooter = new Shooter();
  public static ShooterWrist shooterW = new ShooterWrist();
  public static Intake intake = new Intake();
  public static Climber climber = new Climber();
  public static SysIdRoutine routine;

  // The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(boolean isSim) {

    //Start the USB camera feed
    CameraServer.startAutomaticCapture();

    // Create the SysId routine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((voltage) -> m_robotDrive.driveVolts(voltage.in(Units.Volts)),
        null, // No log consumer, since data is recorded by URCL
        m_robotDrive
      )
    );

    // The methods below return Command objects - map these to buttons when characterizing :)
    /*
    routine.quasistatic(SysIdRoutine.Direction.kForward);
    routine.quasistatic(SysIdRoutine.Direction.kReverse);
    routine.dynamic(SysIdRoutine.Direction.kForward);
    routine.dynamic(SysIdRoutine.Direction.kReverse);
    */

    // SysId config for AdvantageKit
    new SysIdRoutine.Config(
      null, null, null,
      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
    );

    // If statement for Mechanism2D testing - unused
    if (isSim) {
      arm = new ArmSubsystem(
        new ArmSimIO()
      );
      elevatorSim = new ElevatorSimSubsystem(
      new ElevatorSimIO()
      );
    }
    else {
      arm = null;
      elevatorSim = null;
    }
    sim = new MechanismSimulator(arm, elevatorSim);

    // Put auto commands to PathPlanner
    NamedCommands.registerCommand("Shoot Note", new ShootNoteInAuto());
    NamedCommands.registerCommand("IntakeNote", new ReturnHomeAndIntakeInAuto());
    NamedCommands.registerCommand("Shoot Subwoofer", new ShootSubwooferInAuto());
    NamedCommands.registerCommand("Adjust Wrist", new AutoAdjustWristWithIntake());
    NamedCommands.registerCommand("Adjust Subwoofer", new AdjustSubwoofer());
    NamedCommands.registerCommand("Print Position", new PrintPose());
    NamedCommands.registerCommand("Stop Shooter", new StopShooter());
    
    /*
    // Mechanism2D Simulation buttons - mostly for testing ^-^
    SmartDashboard.putData("Arm 0", (Sendable) this.arm.setArmPosition(0));
    SmartDashboard.putData("Arm 1", (Sendable) this.arm.setArmPosition(45));
    SmartDashboard.putData("Arm 2", (Sendable) this.arm.setArmPosition(90));
    SmartDashboard.putData("Arm 3", (Sendable) this.arm.setArmPosition(-90));
    SmartDashboard.putData("Arm 4", (Sendable) this.arm.setArmPosition(270));
    SmartDashboard.putData("Arm 5", (Sendable) this.arm.setArmPosition(180));
    SmartDashboard.putData("Arm 6", (Sendable) this.arm.setArmPosition(220));
    SmartDashboard.putData("Elev 0", (Sendable) this.elevatorSim.setElevatorPosition(Units.inchesToMeters(25)));
    SmartDashboard.putData("Elev 1", (Sendable) this.elevatorSim.setElevatorPosition(Units.inchesToMeters(30)));
    SmartDashboard.putData("Elev 2", (Sendable) this.elevatorSim.setElevatorPosition(Units.inchesToMeters(32)));
    SmartDashboard.putData("Elev 3", (Sendable) this.elevatorSim.setElevatorPosition(Units.inchesToMeters(34.5)));
    */

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
    
    // Build and display the auto chooser using PathPlanner        
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public DriveSubsystem getDrivetrain(){
    return m_robotDrive;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //Driver Buttons
    // m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX()));
    // m_driverController.y().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));
    // m_driverController.a().onTrue(new LimelightDriveToTarget());
    // m_driverController.b().whileTrue(new AdjustAndShootSubwoofer());
    // m_driverController.rightTrigger().onTrue(new ReturnHomeAndIntake());
    // m_driverController.rightBumper().whileTrue(new AdjustAndShootShortDistance());
    // m_driverController.rightBumper().whileFalse(new ParallelCommandGroup(new RunCommand(() -> shooter.setShooterSpeed(0), shooter), new RunCommand(() -> intake.setIntakeSpeed(0, 0, 0), intake)));
    // // Turn these into actual commands eventually
    // //m_driverController.leftTrigger().onFalse(new ParallelDeadlineGroup(new WaitCommand(0.25), new SetIntakeSpeeds(0, -0.1, -0.1)));
    // m_driverController.leftTrigger().whileTrue(new AdjustWristAndFeed());
    // m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new RunCommand(() -> shooter.setShooterSpeed(-1), shooter),
    // new SetIntakeSpeeds(0, -1, 1)));
    // m_driverController.leftBumper().whileFalse(new ParallelCommandGroup(new RunCommand(() -> shooter.setShooterSpeed(0), shooter),
    // new RunCommand(() -> intake.setIntakeSpeed(0, 0, 0), intake)));
    
    // Characterization Controls
    m_driverController.a().onTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.b().onTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.x().onTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    m_driverController.y().onTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));
    
    

    // //Operator buttons
    // m_operatorController.b().whileTrue(new SetClimberDown());
    // m_operatorController.a().whileTrue(new SetClimberUp());
    // m_operatorController.x().onTrue(new ReturnToHome());
    // m_operatorController.povUp().onTrue(new ScoreNoteAmp());
    // //m_operatorController.povLeft().onTrue(new AutoAdjustWristWithIntake());
    // m_operatorController.leftBumper().whileTrue(new RunCommand(() -> shooterW.shooterWristMotor.set(0.1), shooterW));
    // m_operatorController.leftBumper().whileFalse(new RunCommand(() -> shooterW.shooterWristMotor.stopMotor(), shooterW));
    // m_operatorController.rightBumper().whileTrue(new RunCommand(() -> shooterW.shooterWristMotor.set(-0.1), shooterW));
    // m_operatorController.rightBumper().whileFalse(new RunCommand(() -> shooterW.shooterWristMotor.stopMotor(), shooterW));
    // m_operatorController.povLeft().onTrue(new SetElevatorPosition(17.9));
    // m_operatorController.povRight().onTrue(new SetElevatorPosition(RobotConstants.ampElevatorPosition));
    // m_operatorController.leftTrigger().whileTrue(new DriveElevatorUp());
    // m_operatorController.rightTrigger().whileTrue(new DriveElevatorDown());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
