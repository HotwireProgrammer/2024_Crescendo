package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import java.applet.AudioClip;
import java.nio.Buffer;
import java.rmi.server.Operation;
import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.autostep.*;
import frc.robot.swerve.DriveSubsystem;
import frc.robot.swerve.Constants.OIConstants;
import edu.wpi.first.wpilibj.Compressor;
import java.util.*;
import java.util.ResourceBundle.Control;

//import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

	// Joysticks
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick driver;
	// public Joystick flightStickLeft;
	// public Joystick flightStickRight;
	// public CANSparkMax arm = new CANSparkMax(16, MotorType.kBrushless);

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriveSubsystem swerveDrive = new DriveSubsystem();

	// Auto
	public LinkedList<AutoStep> firstAuto;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public TalonSRX one = new TalonSRX(35);
	public TalonSRX two = new TalonSRX(50);
	public TalonSRX three = new TalonSRX(1);
	//public TalonSRX intake = new TalonSRX(19);
	//public CANSparkMax intake = new CANSparkMax(14, MotorType.kBrushless);
	public CANSparkMax intake = new CANSparkMax(14, MotorType.kBrushless);
	public CANSparkMax wench = new CANSparkMax(18, MotorType.kBrushless);
	public DigitalInput limitSwitchOne = new DigitalInput(0);
	public DigitalInput limitSwitchTwo = new DigitalInput(1);

	public boolean holding = false;

	public int climbStep = 0;
	public Timer climbMoveTimer;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {
		SmartDashboard.putNumber("Ballcount", 0);
		SmartDashboard.putBoolean("Balance", false);
		SmartDashboard.putBoolean("DriveStraight", false);
		SmartDashboard.putBoolean("Test", false);
		CameraServer.startAutomaticCapture();
		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		swerveDrive.Init();
	}

	public void disabledInit() {

		// Controllers
		operator = new Joystick(2);
		driver = new Joystick(1);
		// flightStickLeft = new Joystick(3);
		// flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	// fix autonomous spinning

	public void autonomousInit() {
		currentAutoStep = 0;

		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.0f));
		firstAuto.add(new LimelightTrack(swerveDrive, null, limelight));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 1.0f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 90, 0, 1));
		
		autonomousSelected = firstAuto;
		autonomousSelected.get(0).Begin();
		swerveDrive.zeroHeading();
	}


	public void autonomousPeriodic() {

		// autonomous loop
		// System.out.println("Current auto step " + currentAutoStep);
		if (currentAutoStep < autonomousSelected.size()) {

			autonomousSelected.get(currentAutoStep).Update();

			if (autonomousSelected.get(currentAutoStep).isDone) {
				currentAutoStep = currentAutoStep + 1;
				if (currentAutoStep < autonomousSelected.size()) {
					autonomousSelected.get(currentAutoStep).Begin();
				}
			}
		} else {
			// stop drivetrain
			swerveDrive.drive(0, 0, 0, true, true);
		}

	}

	public void teleopInit() {

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		// Controllers
		driver = new Joystick(1);
		operator = new Joystick(2);
		// flightStickLeft = new Joystick(3);
		// flightStickRight = new Joystick(2);

	}

	public void ClimberArmsUp(boolean up) {
		if (up && !limitSwitchOne.get()) {
			//climberOne.set(0.05);
		} else {
			//climberOne.set(0.0);
		}

		if (up && !limitSwitchTwo.get()) {
		//	climberTwo.set(-0.05);
		} else {
	//		climberTwo.set(0.0);
		}
	}

	public void teleopPeriodic() {

		boolean armsUp = false;
		if (operator.getRawButton(4)) {
			armsUp = true;
		}

		if (operator.getRawAxis(3) > 0.01) {
			wench.set(operator.getRawAxis(3));
		} else if (operator.getRawAxis(2) > 0.01) {
			wench.set(-operator.getRawAxis(2));
		} else {
			wench.set(0);
		}

		if (operator.getRawButton(6)) {
			// top
			one.set(TalonSRXControlMode.PercentOutput, -0.50);
			// bottom
			two.set(TalonSRXControlMode.PercentOutput, -0.45);

			System.out.println("one " + one.getSelectedSensorVelocity() + " two " + two.getSelectedSensorVelocity());
		} else {
			one.set(TalonSRXControlMode.PercentOutput, 0);
			two.set(TalonSRXControlMode.PercentOutput, 0);
		}

		if (operator.getRawButton(5)) {
			three.set(TalonSRXControlMode.PercentOutput, 1.0f);
		} else {
			three.set(TalonSRXControlMode.PercentOutput, 0.0);
		}

		if (operator.getRawButton(3)) {
			intake.set(-1.0);
		} else {
			intake.set(0.0);
		}

		// driver

		// limelight climbing
		if (driver.getRawButton(2)) {

			// reset everything for a new climb
			if (driver.getRawButtonPressed(2)) {
				climbStep = 0;
				climbMoveTimer = new Timer();
				climbMoveTimer.reset();
			}

			armsUp = true;

			if (climbStep == 0) {
				// position and rotate robot to on target
				// move up climber arms

				if (limelight.GetAprilID() == 13) {
					limelight.PositionCursor(swerveDrive, 90);
				} else if (limelight.GetAprilID() == 12) {
					limelight.PositionCursor(swerveDrive, -140);
				} else if (limelight.GetAprilID() == 11) {
					limelight.PositionCursor(swerveDrive, -29.5);
				}

				if (limelight.OnTarget()) {
					climbStep = 1;
					climbMoveTimer.start();
				}
			} else if (climbStep == 1) {
				// run wench and move forward
				if (climbMoveTimer.get() < 0.8) {
					swerveDrive.drive(0.1, 0, 0, false, true);
				} else {
					swerveDrive.drive(-0.05, 0.0, 0, false, true);
					wench.set(1.0f);
				}

			}

		} else {

			double pow = 2;
			double axisZero = Math.pow(driver.getRawAxis(0), pow)
					* (driver.getRawAxis(0) / Math.abs(driver.getRawAxis(0)));
			double axisOne = Math.pow(driver.getRawAxis(1), pow)
					* (driver.getRawAxis(1) / Math.abs(driver.getRawAxis(1)));

			if (driver.getRawButton(6)) {
				axisZero = axisZero * 0.25;
				axisOne = axisOne * 0.25;
			}

			swerveDrive.drive(
					MathUtil.applyDeadband(axisZero, OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(axisOne, OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(driver.getRawAxis(4), OIConstants.kDriveDeadband),
					true, true);
		}

		// zero
		if (driver.getRawButton(1)) {
			swerveDrive.zeroHeading();
		}

		// set climber arms
		// ClimberArmsUp(armsUp);
	}

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));

		if (selection == DriveScale.squared) {
			float output = multiplier * (float) (ControllerInput * ControllerInput);

			return output;

		} else if (selection == DriveScale.tangent) {
			return multiplier * (0.4f * (float) Math.tan(1.8 * (multiplier * ControllerInput) - .9) + 0.5f);
		} else if (selection == DriveScale.inverse) {
			return (float) Math.pow(ControllerInput, 1 / 2);
		} else if (selection == DriveScale.cb) {
			return (float) Math.pow(ControllerInput, 3);
		} else if (selection == DriveScale.cbrt) {
			return multiplier * (0.63f * (float) Math.cbrt((multiplier * ControllerInput) - 0.5f) + 0.5f);
		} else {
			return ControllerInput;
		}
	}

	public void testInit() {
		operator = new Joystick(2);
		driver = new Joystick(1);
		// flightStickLeft = new Joystick(3);
		// flightStickRight = new Joystick(2);

		swerveDrive.zeroHeading();
	}

	public void testPeriodic() {
		swerveDrive.drive(
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband),
				MathUtil.applyDeadband(driver.getRawAxis(1), OIConstants.kDriveDeadband),
				-MathUtil.applyDeadband(driver.getRawAxis(0), OIConstants.kDriveDeadband),
				true, false);
	}

	public static float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	public float TranslateController(float input) {
		float deadzone = 0.15f;
		if (input > -deadzone && input < deadzone) {
			input = 0.0f;
		}
		float a = 0.7f;
		float output = (a * input * input * input) + (1 - a) * input;
		return output;
	}
}