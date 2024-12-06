// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorConstants.motorID);
  private final Encoder encoder = new Encoder(0, 1);

  private final PIDController posPID = new PIDController(SmartDashboard.getNumber("PID pos P", Constants.PIDConstants.pos_kp), 
  SmartDashboard.getNumber("PID pos I", Constants.PIDConstants.pos_ki), 
  SmartDashboard.getNumber("PID pos D", Constants.PIDConstants.pos_kd));
  
  

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    motor.setNeutralMode(NeutralMode.Brake);
    encoder.reset();
  }

  public void shoot(double speed){
    double clampedSpeed = MathUtil.clamp(speed, -MotorConstants.maxSpeed, MotorConstants.maxSpeed);
    motor.set(clampedSpeed);
  }

  public double getDistance(){
    return encoder.getDistance() * EncoderConstants.encoderCountsToMeters;
  }

  public void setSpeedRPM(double RPM){
    motor.set(TalonSRXControlMode.Velocity, RPM / 600 * Constants.EncoderConstants.ticksPerRevolution);
  }

  public void setSpeedAngle(double angle){ //angle in degree
    motor.set(TalonSRXControlMode.Position, angle / 360 * Constants.EncoderConstants.ticksPerRevolution);
  }

  public PIDController getPosPID(){
    return posPID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Position: ", getDistance());
  }
}
