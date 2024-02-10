// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FullSystemCommandsTeleop.ReturnToHome;
import frc.robot.commands.FullSystemCommandsTeleop.ScoreNoteTest;
import frc.robot.commands.ShooterWristCommands.ShootNote;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMove;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.armtest.ArmSubsystem;
import frc.robot.subsystems.armtest.io.ArmSimIO;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimSubsystem;
import frc.robot.subsystems.elevator.io.ElevatorSimIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;

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
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static Elevator elevator = new Elevator();
  public static Shooter shooter = new Shooter();
  public static ShooterWrist shooterW = new ShooterWrist();
  public static Intake intake = new Intake();
  public static IntakeMove intakeMove = new IntakeMove();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(boolean isSim) {

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

    //Driver buttons
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new ReturnToHome());
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(new ScoreNoteTest());
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new ShootNote());

    //Operator buttons - TO BE ADDED
          
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
