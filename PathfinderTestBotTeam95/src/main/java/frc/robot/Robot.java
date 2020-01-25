/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.Controller;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  //private DifferentialDrive m_myRobot;
  // private static final int LeftStickY = 1;
  // private static final int RightStickY = 5;

  // private Joystick LeftStick;
  // private Joystick RightStick;

  // private Joystick XBC;

  private XboxController Controller = new XboxController(1);;

  private IMotorControllerEnhanced Left, Right;

  private int leftCanNum = 10;
  private int rightCanNum = 22;

  private double Time = 0;

  // private double timeAtStop = 10;

  @Override
  public void robotInit() {
    Left = new TalonSRX(leftCanNum); //new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    Right = new TalonSRX(rightCanNum);
    // LeftStick = new Joystick(1);
    // RightStick = new Joystick(5);
  }

  @Override
  public void teleopPeriodic() {
    // m_myRobot.tankDrive(LeftStick.getY(), RightStick.getY());
    Left.set(ControlMode.PercentOutput, getLeftY());
    Right.set(ControlMode.PercentOutput, getRightY());
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    // System.out.println("We are in auto periodic");

    // Left.set(ControlMode.PercentOutput, 20);
    // Right.set(ControlMode.Position, 1);
    Time++;
    System.out.println("Time = " + Time);

    if(Time <= 50)
    {
      Left.set(ControlMode.PercentOutput, 1);
    }
    else if(Time >= 50 && Time <= 100)
    {
      Left.set(ControlMode.PercentOutput, 0.5);
    }
    else
    {
      Left.set(ControlMode.Disabled, 0);
    }
  }

  public double getLeftY() {
    return Controller.getY(Hand.kLeft);
  }

  public double getRightY() {
    return Controller.getY(Hand.kRight);
  }

  public double TimeTillStop(double timeTillStop) {
    return timeTillStop = 1;
    // if(Time >= time)
    // {
    //   return 0;
    // }
    // else
    // {
    //   return 10;
    // }
  }
}
