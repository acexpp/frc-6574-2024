package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.armtest.ArmSubsystem;

public class MechanismSimulator {
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d arm;

    private final ArmSubsystem armSubsystem;

    public MechanismSimulator(ArmSubsystem armS) {
        //this.armSubsystem = armS;

        //this.panel = new Mechanism2d(100, 0);
    }
}
