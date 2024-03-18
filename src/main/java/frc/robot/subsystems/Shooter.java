// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public TalonFX kShooterLeft;
  public TalonFX kShooterRight;
  // public TalonFXConfiguration shooterAngleFxConfiguration = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfiguration = new TalonFXConfiguration();
  public CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
    /* Start at velocity 0, no feed forward, use slot 1 */
  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(
    0, 
    0, 
    0, 
    1, 
    false, false, false);

  
  /** Creates a new Shooter. */
  public Shooter() {
    kShooterLeft = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kShooterLeft.getConfigurator().apply(shooterVelocityFxConfiguration);
    kShooterLeft.setNeutralMode(NeutralModeValue.Brake);
    kShooterLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));

    kShooterRight = new TalonFX(Constants.RobotConstants.shooterRightCANID);
    kShooterRight.getConfigurator().apply(shooterVelocityFxConfiguration);
    kShooterRight.setNeutralMode(NeutralModeValue.Brake);
    kShooterRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(25));
    
    shooterVelocityFxConfiguration.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    shooterVelocityFxConfiguration.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    shooterVelocityFxConfiguration.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity", getVelocity());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    kShooterLeft.set(-speed);
    kShooterRight.set(-speed);
  }

  public double getVelocity(){
    return kShooterLeft.getVelocity().getValue();
  }
  
  /* thinking we only really need to use this if we don't want full speed for the shooter... who knows
  
  public void setShooterVelocity(double desiredRotationsPerSecond) {
    kShooterLeft.setControl(m_torqueVelocity.withVelocity(-desiredRotationsPerSecond).withFeedForward(-1));
  }

  */
}
