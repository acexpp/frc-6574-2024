/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//THIS CLASS MAY NOT BE ACCURATE - DEFINITE TUNING NEEDED :)))
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer; //Might remove, doesn't cause errors yet

import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public CANSparkMax leftMotor;
  public CANSparkMax rightMotor;
  private SparkLimitSwitch climberReverseLimit;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private double maxSpeed = 0.4;
  //private float maxElevatorExtension = 32.5f;
  
  public Climber() {
    leftMotor = new CANSparkMax(Constants.RobotConstants.climberLeftCANID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.RobotConstants.climberRightCANID, MotorType.kBrushless);

    climberReverseLimit = leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(25);
    rightMotor.setSmartCurrentLimit(25);

    leftMotor.setInverted(false);

    rightMotor.follow(leftMotor, true);

    //leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    //leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, maxElevatorExtension);

    //leftMotor.getEncoder().setPosition(0);

   /*  climberPIDController = leftMotor.getPIDController();
    leftMotor.getEncoder();

    kP = 0.15;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = .6;
    kMinOutput = -.6;

    climberPIDController.setP(kP);
    climberPIDController.setI(kI);
    climberPIDController.setD(kD);
    climberPIDController.setIZone(kIz);
    climberPIDController.setFF(kFF);
    climberPIDController.setOutputRange(kMinOutput, kMaxOutput);*/
  }


  public void driveClimber(double speed) {
    leftMotor.set(speed * maxSpeed);
  }

 /*public void setPosition(double position) {
    climberPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }*/

  @Override
  public void periodic() {

    climberReverseLimit.enableLimitSwitch(true);
    SmartDashboard.putNumber("Climber encoder", leftMotor.getEncoder().getPosition());

    /*     if (RobotContainer.operator.getRawButtonPressed(3)) {
      leftMotor.set(.15);
    }
      else if (RobotContainer.operator.getRawButtonReleased(3)) {
        leftMotor.set(0);
      }

      if (RobotContainer.operator.getRawButtonPressed(4)) {
        leftMotor.set(-.15);
      }
      else if (RobotContainer.operator.getRawButtonReleased(4)) {
        leftMotor.set(0);
      } */

    }


    // This method will be called once per scheduler run
  }
  
