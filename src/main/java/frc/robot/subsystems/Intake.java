// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  public CANSparkMax kIntakeLinkageLeft;
  public CANSparkMax kIntakeLinkageRight;
  public CANSparkMax kIntakeRollerBottom;
  public CANSparkMax kIntakeRollerTop;
  
  /** Creates a new Intake. */
  public Intake() {
    kIntakeLinkageLeft = new CANSparkMax(Constants.RobotConstants.kIntakeLinkageLeftCanId, MotorType.kBrushless);
    kIntakeLinkageLeft.restoreFactoryDefaults();
    kIntakeLinkageLeft.setIdleMode(IdleMode.kBrake);
    kIntakeLinkageLeft.setSmartCurrentLimit(25);

    kIntakeLinkageRight = new CANSparkMax(Constants.RobotConstants.kIntakeLinkageRightCanId, MotorType.kBrushless);
    kIntakeLinkageRight.restoreFactoryDefaults();
    kIntakeLinkageRight.setIdleMode(IdleMode.kBrake);
    kIntakeLinkageRight.setSmartCurrentLimit(25);

    kIntakeRollerBottom = new CANSparkMax(Constants.RobotConstants.kIntakeRollerBottomCanId, MotorType.kBrushless);
    kIntakeRollerBottom.restoreFactoryDefaults();
    kIntakeRollerBottom.setIdleMode(IdleMode.kBrake);
    kIntakeRollerBottom.setSmartCurrentLimit(25);

    kIntakeRollerTop = new CANSparkMax(Constants.RobotConstants.kIntakeRollerTopCanId, MotorType.kBrushless);
    kIntakeRollerTop.restoreFactoryDefaults();
    kIntakeRollerTop.setIdleMode(IdleMode.kBrake);
    kIntakeRollerTop.setSmartCurrentLimit(25);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    kIntakeLinkageLeft.set(speed);
    kIntakeLinkageRight.set(speed);
    kIntakeRollerBottom.set(speed);
    kIntakeRollerTop.set(speed);

  }

  public void setOutakeSpeed() {
    kIntakeLinkageLeft.set(-1);
    kIntakeLinkageRight.set(-1);
    kIntakeRollerBottom.set(-1);
    kIntakeRollerTop.set(-1);

  }
}
