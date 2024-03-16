// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation2d;*/

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot{

  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  String rangeTarget;

  double setpoint;

  double motorPowerSet;
  double motorPowerPID;

  AnalogInput analog = new AnalogInput(1);

  Joystick controller = new Joystick(0);

  double speed = 0;

  //PhotonCamera cam = new PhotonCamera("photonvision");

  WPI_TalonSRX arm1 = new WPI_TalonSRX(20);
  WPI_TalonSRX arm2 = new WPI_TalonSRX(21);

  PWMSparkMax intakee = new PWMSparkMax(0);

  VictorSP shooter1 = new VictorSP(1);
  VictorSP shooter2 = new VictorSP(2);
  
  WPI_VictorSPX backright = new WPI_VictorSPX(23);
  WPI_VictorSPX frontright = new WPI_VictorSPX(22);
  WPI_TalonSRX frontleft = new WPI_TalonSRX(24);
  WPI_VictorSPX backleft = new WPI_VictorSPX(25);

  MecanumDrive m_robotDrive = new MecanumDrive(frontleft, backleft, frontright, backright);

  @Override
  public void robotInit() {
    frontright.setInverted(true);
    backright.setInverted(true);
    gyro.reset();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    Rotation2d rotation2d = gyro.getRotation2d();

    System.out.println(rotation2d);

    String desiredID = SmartDashboard.getString("Desired ID", null);

    //var result = cam.getLatestResult();

    double leftAxis = controller.getRawAxis(2);
    double rightAxis = controller.getRawAxis(3);

    String intakeStatus;

    double analogValue = analog.getValue();
    
    SmartDashboard.putNumber("Test Random",Math.random());

    double motorPower = rightAxis+(leftAxis*-1);

    arm1.set(motorPower*-1);
    arm2.set(motorPower*-1);

    m_robotDrive.driveCartesian(-controller.getRawAxis(1), -controller.getRawAxis(0), controller.getRawAxis(4), rotation2d);

    /*System.out.println(analogValue);*/

    if (controller.getRawButton(4) && speed < 1) {
      speed += 0.01;
      shooter1.set(speed*-1);
      shooter2.set(speed);
      } else if(controller.getRawButtonReleased(4)) {
        shooter1.set(0);
        shooter2.set(0);
        speed =0;
      }

    if (controller.getRawButtonPressed(3)){
      intakee.set(0.5);
      System.out.println("Intake Working");
      intakeStatus = "Intake Working";
      SmartDashboard.putString("Intake Status", intakeStatus);
    }
    if(controller.getRawButtonReleased(3)) {
      intakee.set(0);
      System.out.println("Intake Stopped");
      intakeStatus = "Intake Stopped";
      SmartDashboard.putString("Intake Status", intakeStatus);
    }
    
    if(controller.getRawButtonPressed(1)){
      intakee.set(-0.18);
      shooter1.set(0.2);
      shooter2.set(-0.2);
    }
    if(controller.getRawButtonReleased(1)){
      intakee.set(0);
      shooter1.set(0);
      shooter2.set(0);
    }

    if (controller.getRawButton(7)){
      setpoint = 235;
      double k = 0.005;
      double error = analogValue - setpoint;

      arm1.set(MathUtil.clamp(error*k, -0.7, 0.45));
      arm2.set(MathUtil.clamp(error*k, -0.7, 0.45));
    }

    /*if (result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      int targetID = target.getFiducialId();
      if (targetID == Integer.valueOf(desiredID)){
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();

        System.out.format("Target ID",  desiredID ,"detected, range is: ", bestCameraToTarget);
      } else {
        System.out.format("Apriltag ID", targetID, "detected but range is not calculated");
      }
    } else {
      System.out.println("No Apriltag Detected");
    }*/
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}