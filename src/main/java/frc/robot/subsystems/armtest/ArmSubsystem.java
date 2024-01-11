package frc.robot.subsystems.armtest;

import java.util.Optional;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armtest.io.ArmIO;

public class ArmSubsystem extends SubsystemBase{

    private final ArmIO io;

    private final ProfiledPIDController controller;
    private final ArmFeedforward ff;

    private ArmState.InputState currentState;

    private ArmState.GoalState goal = new ArmState.GoalState(45);

    public ArmSubsystem(ArmIO armIO) {
        this.io = armIO;

        this.controller = new ProfiledPIDController(0, 0, 0, new Constraints(null, null));
        this.ff = new ArmFeedforward(0, 0, 0);
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            currentState = this.io.getState();

            double effort = this.controller.calculate(currentState.currentAngleDegrees(), goal.position());
            double feedforward =  this.ff.calculate(
                Units.degreesToRadians(currentState.currentAngleDegrees()), 
                Units.degreesToRadians(currentState.currentVelocityDegreesPerSecond())
            );

            effort += feedforward;

            this.io.setState(
                new ArmState.OutputState(Optional.of(effort))
            );
        }
    }

    public ArmState.InputState getState() {
        return this.currentState;
    }

    public Command setArmPosition(double degrees) {
        return new InstantCommand(() -> this.goal = new ArmState.GoalState(degrees));
    }
    
}
