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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

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

  private IMotorControllerEnhanced Left, LFollower1, LFollower2, Right, RFollower1, RFollower2;

  private Faults Faults = new Faults();

  private int leftCanNum = 10;
  private int lFollowerCanNum1 = 11;
  private int lFollowerCanNum2 = 12;
  private int rightCanNum = 20;
  private int rFollowerCanNum1 = 21;
  private int rFollowerCanNum2 = 22;

  private double Time = 0;

  private int PID_IDX = 0;
  private int CAN_TIMEOUT_MS = 10;


  private double Encoder_Pos = getPositionInches();
  private double Left_To_Right_Offset_Inches;
  private double Top_Speed;
  public int Right_Encoder_Pos = 0;
  public int Left_Encoder_Pos = 0;
  private double wheel_diameter = 5*0.0254;
  public static double outputLeft, outputRight;
  public static int whatPath;
  public static int spin = 0;

  // private double timeAtStop = 10;

  @Override
  public void robotInit() {
    Left = new TalonSRX(leftCanNum); //new DifferentialDrive(new PWMVictorSPX(0), new PWMVictorSPX(1));
    LFollower1 = new TalonSRX(lFollowerCanNum1);
    LFollower2 = new TalonSRX(lFollowerCanNum2);
    Right = new TalonSRX(rightCanNum);
    RFollower1 = new TalonSRX(rFollowerCanNum1);
    RFollower2 = new TalonSRX(rFollowerCanNum2);

    LFollower1.set(ControlMode.Follower, leftCanNum);
    LFollower2.set(ControlMode.Follower, leftCanNum);
    RFollower1.set(ControlMode.Follower, rightCanNum);
    RFollower2.set(ControlMode.Follower, rightCanNum);

    Left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_IDX, CAN_TIMEOUT_MS);
    Left.setSensorPhase(true);
    // LeftStick = new Joystick(1);
    // RightStick = new Joystick(5);
  }

  @Override
  public void teleopPeriodic() {

    System.out.println("Right Y value = " + getRightY() + ", left Y value = " + getLeftY());

    // m_myRobot.tankDrive(LeftStick.getY(), RightStick.getY());
    Left.set(ControlMode.PercentOutput, getLeftY());
    Right.set(ControlMode.PercentOutput, getRightY());
  }

  @Override
  public void autonomousInit() {
    Left.setInverted(false);

    PathFinderCommand(false, false, 24, 1);
  }

  @Override
  public void autonomousPeriodic() {

    outputLeft = leftEncFollower.calculate(Left_Encoder_Pos);
    outputRight = rightEncFollower.calculate(Right_Encoder_Pos);

    Left.getFaults(Faults);

    Left.set(ControlMode.PercentOutput, outputLeft);
    Right.set(ControlMode.PercentOutput, outputRight);
    // System.out.println("We are in auto periodic");

    // Left.set(ControlMode.PercentOutput, 20);
    // Right.set(ControlMode.Position, 1);
    // Time++;
    // System.out.println("Time = " + Time);

    // if(Time <= 50)
    // {
    //   Left.set(ControlMode.PercentOutput, 1);
    // }
    // else if(Time >= 50 && Time <= 100)
    // {
    //   Left.set(ControlMode.PercentOutput, 0.5);
    // }
    // else
    // {
    //   Left.set(ControlMode.Disabled, 0);
    // }
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

  public void log() {
    
  }

  public double getPositionInches() {
    return Left.getSelectedSensorPosition(PID_IDX);
  }

  // All of these are the possible path values
  public static int ForwardTenFeet = 1;

  private static boolean LeftOrRight;
  private static boolean WhatGearAreWeIn;

  public static EncoderFollower leftEncFollower;
  public static EncoderFollower rightEncFollower;

  public static Waypoint[] points;

  public void PathFinderCommand(boolean RightOrLeft, boolean WhichGearAreWeIn, double a, int whatPath) {
    //requires(Robot.drivebase);
    //requires(Robot.pathfinder);
    System.out.println("We are in PathfinderCommand");
    // this.setInterruptible(false);
    //System.out.println("we are in the constructor");
    this.LeftOrRight = RightOrLeft;
    this.WhatGearAreWeIn = WhichGearAreWeIn;
    this.Left_To_Right_Offset_Inches = a;
    this.whatPath = whatPath;

    if (whatPath == ForwardTenFeet)
      {
        points = new Waypoint[] {
          new Waypoint(0, 0, 0),
          new Waypoint(10, 0, 0)
        };
      }

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    // Wheelbase Width = 0.5m
    TankModifier modifier = new TankModifier(trajectory).modify(0.5);

    // Do something with the new Trajectories...
    //Trajectory left = modifier.getLeftTrajectory();
    //Trajectory right = modifier.getRightTrajectory();

    leftEncFollower = new EncoderFollower(modifier.getLeftTrajectory());
    rightEncFollower = new EncoderFollower(modifier.getRightTrajectory());

    // Determine whether the encoder position is the left or right encoder position.
    if(LeftOrRight)
    {
      Left_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
      Right_Encoder_Pos = (int) (Math.round(Encoder_Pos));
    }
    else
    {
      Left_Encoder_Pos = (int) (Math.round(Encoder_Pos));
      Right_Encoder_Pos = (int) (Math.round(Encoder_Pos) + Left_To_Right_Offset_Inches);
    }

    leftEncFollower.configureEncoder(Left_Encoder_Pos, 1000, wheel_diameter);
    rightEncFollower.configureEncoder(Right_Encoder_Pos, 1000, wheel_diameter);

    // Determine what gear we are in, then make the top speed here equal to the top speed for the gear.
    if(WhatGearAreWeIn)
    {
      Top_Speed = 1;
    }
    else
    {
      Top_Speed = 2;
    }

    leftEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
    rightEncFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Top_Speed, 0);
  }

}
