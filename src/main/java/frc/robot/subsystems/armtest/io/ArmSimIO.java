package frc.robot.subsystems.armtest.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.armtest.ArmState;
import frc.robot.subsystems.armtest.ArmState.InputState;
import frc.robot.subsystems.armtest.ArmState.OutputState;

public class ArmSimIO implements ArmIO{

    //Creates a simulated single-jointed arm
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 50, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(14), 
    Units.lbsToKilograms(5)), Units.inchesToMeters(14), -2 * Math.PI, 2 * Math.PI, true, 0);

    @Override
    public InputState getState() {
        sim.update(0.02);
        return new ArmState.InputState(
            Units.radiansToDegrees(sim.getAngleRads()), 
            Units.radiansToDegrees(sim.getVelocityRadPerSec()));
    }

    @Override
    public void setState(OutputState output) {
        output.voltage().ifPresent((volts) -> {
            sim.setInputVoltage(volts);
        });
    }
}
