// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorConstants.motorID);
  private final Encoder encoder = new Encoder(0, 1);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Position: ", getDistance());
  }
}
