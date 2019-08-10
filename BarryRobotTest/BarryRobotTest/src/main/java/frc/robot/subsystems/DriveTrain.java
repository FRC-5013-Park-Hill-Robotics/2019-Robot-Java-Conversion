/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX leftMotor1;
  private WPI_TalonSRX leftMotor2;
  private WPI_TalonSRX rightMotor1;
  private WPI_TalonSRX rightMotor2;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private DifferentialDrive mainDrive;

  public DriveTrain(){
  
    this.leftMotor1 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_1_ID);
    this.leftMotor2 = new WPI_TalonSRX(RobotMap.LEFT_MOTOR_2_ID);
    this.rightMotor1 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_1_ID);
    this.rightMotor2 = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR_2_ID);

    this.leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);
    this.rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);

    this.mainDrive = new DifferentialDrive(leftMotors, rightMotors);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new ArcadeDrive());

  }

public void arcadeDrive(double throttle, double rotation){
  this.mainDrive.arcadeDrive(throttle, rotation);



}



}
