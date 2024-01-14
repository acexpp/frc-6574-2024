package frc.robot.subsystems.elevator.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorState.InputState;
import frc.robot.subsystems.elevator.ElevatorState.OutputState;

public class ElevatorSimIO implements ElevatorIO{

    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getNEO(1), 
        50, Units.lbsToKilograms(35), 
        Units.inchesToMeters(0.8), Units.inchesToMeters(5), 
        Units.inchesToMeters(40), true,
        Units.inchesToMeters(5));

    @Override
    public InputState getState() {
        sim.update(0.02);
        return new ElevatorState.InputState(
            sim.getPositionMeters(),
            sim.getVelocityMetersPerSecond()
        );
    }

    @Override
    public void setState(OutputState output) {
        output.voltage().ifPresent((volts) -> {
        sim.setInputVoltage(volts);
        });
    }

}
