package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.armtest.ArmState;
import frc.robot.subsystems.armtest.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class MechanismSimulator {
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d elevator;
    

    private final ArmSubsystem armSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public MechanismSimulator(ArmSubsystem armS, ElevatorSubsystem elevatorS) {
        this.armSubsystem = armS;
        this.elevatorSubsystem = elevatorS;

        this.panel = new Mechanism2d(Units.inchesToMeters(100), Units.inchesToMeters(100));
        this.root = panel.getRoot("daroot", Units.inchesToMeters(7.35), Units.inchesToMeters(10));
        this.elevator = root.append(
            new MechanismLigament2d("elevator", Units.inchesToMeters(10), 90, 10, new Color8Bit(Color.kRed))
        );
        
        this.arm = elevator.append(
            new MechanismLigament2d("arm", Units.inchesToMeters(25), 0, 6, new Color8Bit(Color.kYellow))
        );
    }

    public void periodic() {
        ArmState.InputState currentState = armSubsystem.getState();
        if (currentState != null) {
            this.arm.setAngle(currentState.currentAngleDegrees());
        }
        ElevatorState.InputState currentState2 = elevatorSubsystem.getState();
        if (currentState2 != null) {
            this.elevator.setLength(currentState2.currentHeightMeters());
        }
        Logger.recordOutput("Robot Simulation", panel);
    }
}
