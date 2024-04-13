// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public TalonFX kShooterLeft;
  public TalonFX kShooterRight;
  // public TalonFXConfiguration shooterAngleFxConfiguration = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationL = new TalonFXConfiguration();
  public TalonFXConfiguration shooterVelocityFxConfigurationR = new TalonFXConfiguration();
  public CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();
  double mvarShooterSpinupStartTime=0;

    /* Start at velocity 0, no feed forward, use slot 1 */ //maybe disregard the previous comment, i don't remember what it means
  private final VelocityDutyCycle m_torqueVelocity = new VelocityDutyCycle(
    0, 
    0, 
    false, 0, 
    1, 
    false, false, false
    );

  private final MotionMagicVelocityVoltage m_motionMagicVelocity = new MotionMagicVelocityVoltage(
    0, 
    800, 
    false, 0, 
    0, false, 
    false, false
    );
    
  /** Creates a new Shooter. */
  public Shooter() {
    //configuration settings, slot 1 might get booted?
    /* 
    shooterVelocityFxConfiguration.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    shooterVelocityFxConfiguration.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    shooterVelocityFxConfiguration.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    */

    // TODO: Needs tuning when I get the robot back
    shooterVelocityFxConfigurationL.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationL.Slot0.kV = 0.14; //0.125 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationL.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationL.Slot0.kP = 0.44; //0.44 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationL.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationL.Slot0.kD = 0; //0.1 // no output for error derivative

    shooterVelocityFxConfigurationR.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
    shooterVelocityFxConfigurationR.Slot0.kV = 0.14; //0.132 // A velocity target of 1 rps results in 0.12 V output
    shooterVelocityFxConfigurationR.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    shooterVelocityFxConfigurationR.Slot0.kP = 0.44; //0.88 // An error of 1 rps results in 0.11 V output
    shooterVelocityFxConfigurationR.Slot0.kI = 0; // no output for integrated error
    shooterVelocityFxConfigurationR.Slot0.kD = 0; // no output for error derivative

    var motionMagicConfigsL = shooterVelocityFxConfigurationL.MotionMagic;
    var motionMagicConfigsR = shooterVelocityFxConfigurationR.MotionMagic;
    motionMagicConfigsL.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsL.MotionMagicJerk = 0; //16000 
    motionMagicConfigsR.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigsR.MotionMagicJerk = 0; 
    
    //actually the motors
    kShooterLeft = new TalonFX(Constants.RobotConstants.shooterLeftCANID);
    kShooterLeft.getConfigurator().apply(shooterVelocityFxConfigurationL);
    kShooterLeft.setNeutralMode(NeutralModeValue.Coast);
    kShooterLeft.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterLeft.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));

    kShooterRight = new TalonFX(Constants.RobotConstants.shooterRightCANID);
    kShooterRight.getConfigurator().apply(shooterVelocityFxConfigurationR);
    kShooterRight.setNeutralMode(NeutralModeValue.Coast);
    kShooterRight.getConfigurator().apply(currentLimitConfig.withStatorCurrentLimit(90));
    kShooterRight.getConfigurator().apply(currentLimitConfig.withSupplyCurrentLimit(50));
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity (L)", kShooterLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Velocity (R)", kShooterRight.getVelocity().getValueAsDouble());
  }

  public void setShooterSpeed(double speed) {
    mvarShooterSpinupStartTime=Timer.getFPGATimestamp();
    kShooterLeft.set(-speed);
    kShooterRight.set(-speed);
  }

  public double getVelocity(){
    return kShooterLeft.getVelocity().getValue();
  }
  
  public void setShooterVelocity(double desiredRotationsPerSecond) {
    kShooterLeft.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
    kShooterRight.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(1));
  }

  public void setShooterVelocityUsingMotionMagic(double desiredVelocityMotionMagic) {
    kShooterLeft.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
    kShooterRight.setControl(m_motionMagicVelocity.withVelocity(desiredVelocityMotionMagic));
  }
}
