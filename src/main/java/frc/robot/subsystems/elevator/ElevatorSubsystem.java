package frc.robot.subsystems.elevator;

import java.util.Optional;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armtest.ArmState;
import frc.robot.subsystems.elevator.ElevatorState.GoalState;
import frc.robot.subsystems.elevator.io.ElevatorIO;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final ElevatorIO io;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;

    private ElevatorState.InputState currentState;

    private ElevatorState.GoalState goal = new GoalState(Units.inchesToMeters(20));

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.io = elevatorIO;

        this.controller = new ProfiledPIDController(10, 0, 0, new Constraints(100, 100));
        this.ff = new ElevatorFeedforward(0, 2.6057, 0);
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            currentState = this.io.getState();

            double effort = this.controller.calculate(currentState.currentHeightMeters(), goal.position());
            double feedforward = this.ff.calculate(
                currentState.currentHeightMeters(),
                currentState.currentVelocityMetersPerSecond()
            );

            effort += feedforward;

            this.io.setState(
                new ElevatorState.OutputState(Optional.of(effort))
            );
        }
    }

    public ElevatorState.InputState getState() {
        return this.currentState;
    }

    public Command setElevatorPosition(double meters) {
        return new InstantCommand(() -> this.goal = new ElevatorState.GoalState(meters));
    }
}
