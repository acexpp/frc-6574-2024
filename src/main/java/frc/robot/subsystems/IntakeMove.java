// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMove extends SubsystemBase {

  public CANSparkMax intakeMoveLeft;

  public CANSparkMax intakeMoveRight;
  //private RelativeEncoder wristEncoder;

  private SparkLimitSwitch intakeReverseLimit;
  private SparkPIDController intakeMoveLeftPidController;
  private SparkPIDController intakeMoveRightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  //private double maxSpeed = 0.25;
  //private double deadBand = 0.1;

  /** Creates a new IntakeMove. */
  public IntakeMove() {
    intakeMoveLeft = new CANSparkMax(Constants.RobotConstants.intakeMoveLeftCANID, MotorType.kBrushless);
    intakeMoveRight = new CANSparkMax(Constants.RobotConstants.intakeMoveRightCANID, MotorType.kBrushless);

    intakeReverseLimit = intakeMoveLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    intakeMoveLeft.restoreFactoryDefaults();
    intakeMoveRight.restoreFactoryDefaults();

    intakeMoveLeft.setIdleMode(IdleMode.kBrake);
    intakeMoveRight.setIdleMode(IdleMode.kBrake);

    intakeMoveLeft.setSmartCurrentLimit(25);
    intakeMoveRight.setSmartCurrentLimit(25);

    intakeMoveLeft.setInverted(false);

    intakeMoveRight.follow(intakeMoveLeft, true);

    intakeMoveLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    //intakeMoveLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, maxIntakeExtension);

    intakeMoveLeft.getEncoder().setPosition(0);

    intakeMoveLeftPidController = intakeMoveLeft.getPIDController();
    intakeMoveLeft.getEncoder();

    kP = 2.8; //2.5 last working value
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .5;
    kMinOutput = -.5;

    intakeMoveLeftPidController.setP(kP);
    intakeMoveLeftPidController.setI(kI);
    intakeMoveLeftPidController.setD(kD);
    intakeMoveLeftPidController.setIZone(kIz);
    intakeMoveLeftPidController.setFF(kFF);
    intakeMoveLeftPidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeMoveLeftPidController.setPositionPIDWrappingEnabled(true);
    intakeMoveLeftPidController.setPositionPIDWrappingMinInput(0);
    intakeMoveLeftPidController.setPositionPIDWrappingMaxInput(1);
  }

  @Override

  public void periodic() {
    SmartDashboard.putNumber("Intake Position", getEncoderPositionLeft());
    //SmartDashboard.putNumber("Wrist Joystick", RobotContainer.operator.getRawAxis(5));
    //SmartDashboard.putNumber("Wrist encoder", wristMotor.getEncoder().getPosition());

    /* if (RobotContainer.operator.getRawAxis(5) > deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } else if (RobotContainer.operator.getRawAxis(5) < -deadBand) {
      wristMotor.set(-RobotContainer.operator.getRawAxis(5) * maxSpeed);
    } */



    /* if (RobotContainer.operator.getRawButtonPressed(1)) {
      intakeMotor.set(1);
    }
      else if (RobotContainer.operator.getRawButtonReleased(1)) {
        intakeMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(2)) {
        intakeMotor.set(-1);
      }
      else if (RobotContainer.operator.getRawButtonReleased(2)) {
        intakeMotor.set(0);
      } */


    }




  public void setSpeed(double speed)
  {
    intakeMoveLeft.set(speed);
  }

  public void stop()
  {
    intakeMoveLeft.stopMotor();
    intakeMoveRight.stopMotor();
  }


  public double getEncoderPositionLeft() {
    return intakeMoveLeft.getEncoder().getPosition();
  }

  public double getEncoderPositionRight() {
    return intakeMoveRight.getEncoder().getPosition();
  }

  public void setPosition(double position) {
    intakeMoveLeftPidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

}
