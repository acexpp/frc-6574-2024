// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.AutoFullSystemCommands.IntakeInAuto;
import frc.robot.commands.AutoFullSystemCommands.LimelightDriveToTarget;
import frc.robot.commands.AutoFullSystemCommands.ShootNoteInAuto;
import frc.robot.commands.ClimberCommands.SetClimberDown;
import frc.robot.commands.ClimberCommands.SetClimberUp;
import frc.robot.commands.ElevatorCommands.SetElevatorDown;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorUp;
import frc.robot.commands.FullSystemCommandsTeleop.IntakeNoteFromFloor;
import frc.robot.commands.FullSystemCommandsTeleop.ReturnToHome;
import frc.robot.commands.FullSystemCommandsTeleop.ScoreNoteTest;
//import frc.robot.commands.IntakeMoveCommands.SetIntakeMovePosition;
import frc.robot.commands.ShooterWristCommands.SetShooterWristPosition;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMove;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

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
  //public static IntakeMove intakeMove = new IntakeMove();
  public static Climber climber = new Climber();
  public static SysIdRoutine routine;
  //public static Rev2mDistanceSensor sensor = new Rev2mDistanceSensor(Port.kOnboard); //change later?
  

  // The driver's controller
  public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(boolean isSim) {

    // Create the SysId routine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism((voltage) -> m_robotDrive.driveVolts(voltage.in(Units.Volts)),
        null, // No log consumer, since data is recorded by URCL
        m_robotDrive
      )
    );

    // The methods below return Command objects
    /*
    routine.quasistatic(SysIdRoutine.Direction.kForward);
    routine.quasistatic(SysIdRoutine.Direction.kReverse);
    routine.dynamic(SysIdRoutine.Direction.kForward);
    routine.dynamic(SysIdRoutine.Direction.kReverse);
    */

    // AdvantageKit users should log the test state using the following configuration
    new SysIdRoutine.Config(
      null, null, null,
      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
    );

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

    //sensor.setRangeProfile(RangeProfile.kHighSpeed);

    //SmartDashboard.putNumber("Sensor data", sensor.getRange());
    NamedCommands.registerCommand("Shoot Note", new ShootNoteInAuto());
    NamedCommands.registerCommand("Intake Note", new IntakeInAuto());
    
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

    

    //debug tab and visual for gyro
    ShuffleboardTab teleOpTab = Shuffleboard.getTab("TeleOp");
    teleOpTab.addDouble("Gyro", m_robotDrive::getHeading);

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
    //Driver Buttons - WIP
    /* 
    m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX()));
    m_driverController.y().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading()));
    //m_driverController.rightBumper().whileTrue(new ShootNote());
    m_driverController.a().onTrue(new LimelightDriveToTarget());
    
    m_driverController.rightBumper().whileTrue(new ParallelCommandGroup(
      new RunCommand(() -> shooter.setShooterSpeed(-Constants.RobotConstants.shooterSpeed), shooter),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new RunCommand(() -> intake.setIntakeSpeed(-1, -1), intake)
      )
    ));
    m_driverController.rightBumper().whileFalse(new ParallelCommandGroup(
      new RunCommand(() -> shooter.setShooterSpeed(0), shooter), 
      new RunCommand(() -> intake.setIntakeSpeed(0, 0), intake)
    ));
    */
    m_driverController.a().onTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    m_driverController.b().onTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
    m_driverController.x().onTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    m_driverController.y().onTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));

    //Operator buttons - WIP
    /* 
    m_operatorController.leftBumper().whileTrue(new IntakeNote());
    m_operatorController.rightBumper().whileTrue(new RunCommand(() -> intake.setOutakeSpeed(), intake));
    m_operatorController.rightBumper().whileTrue(new RunCommand(() -> intake.setIntakeSpeed(0, 0), intake));
    m_operatorController.b().whileTrue(new SetClimberDown());
    m_operatorController.a().whileTrue(new SetClimberUp());
    m_operatorController.x().onTrue(new SetElevatorPosition(8.16));
    m_operatorController.y().onTrue(new SetElevatorPosition(0));
    m_operatorController.povUp().onTrue(new SetElevatorPosition(15));
    //m_operatorController.y().onTrue(new IntakeNoteFromFloor());
    //m_operatorController.x().onTrue(new SetShooterWristPosition(RobotConstants.shooterWristTestPos));
    m_operatorController.povLeft().whileTrue(new RunCommand(() -> shooterW.setSpeed(0.1), shooterW));
    m_operatorController.povLeft().whileFalse(new RunCommand(() -> shooterW.setSpeed(0), shooterW));
    m_operatorController.povRight().whileTrue(new RunCommand(() -> shooterW.setSpeed(-0.1), shooterW));
    m_operatorController.povRight().whileFalse(new RunCommand(() -> shooterW.setSpeed(0), shooterW));
    */
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
