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
	public Joystick flightStickLeft;
	public Joystick flightStickRight;
	// public CANSparkMax arm = new CANSparkMax(16, MotorType.kBrushless);

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	// public SwerveModule swerveOne = new SwerveModule(50, 49);
	// public SwerveModule swerveTwo = new SwerveModule(34, 44);
	// public SwerveModule swerveThree = new SwerveModule(32, 42);
	// public SwerveModule swerveFour = new SwerveModule(31, 41);
	public DriveSubsystem swerveDrive = new DriveSubsystem();

	public DriverStation driverStation;
	public RobotState currentState;
	public boolean DriveEnabled;
	public boolean SpinEnabled;

	// Auto
	public LinkedList<AutoStep> firstAuto;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public DoubleSolenoid solenoid1 = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 8, 9);
	public DoubleSolenoid solenoid2 = new DoubleSolenoid(50, PneumaticsModuleType.REVPH, 10, 11);

	public CANSparkMax motorGripper = new CANSparkMax(18, MotorType.kBrushless);
	public CANSparkMax armExtend = new CANSparkMax(19, MotorType.kBrushless);

	public boolean holding = false;

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
	}

	public void disabledInit() {

		// Controllers
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {
		/*
		 * currentAutoStep = 0;
		 * NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").
		 * setNumber(1);
		 * 
		 * operator = new Joystick(1);
		 * 
		 * limelight.SetLight(true);
		 * 
		 * // Autonomous selection
		 * 
		 * double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);
		 * 
		 * // firstAuto.add(new TimedForward(driveTrain, 1, 0.2f));
		 * firstAuto = new LinkedList<AutoStep>();
		 * 
		 * if (autoChoice == 0) {
		 * autonomousSelected = firstAuto;
		 * }
		 * autonomousSelected.get(0).Begin();
		 */
	}

	public void autonomousPeriodic() {

		solenoid1.set(Value.kReverse);
		solenoid2.set(Value.kReverse);

		/*
		 * SmartDashboard.putBoolean("RobotEnabled", true);
		 * 
		 * // autonomous loop
		 * // System.out.println("Current auto step " + currentAutoStep);
		 * if (currentAutoStep < autonomousSelected.size()) {
		 * 
		 * autonomousSelected.get(currentAutoStep).Update();
		 * 
		 * if (autonomousSelected.get(currentAutoStep).isDone) {
		 * currentAutoStep = currentAutoStep + 1;
		 * if (currentAutoStep < autonomousSelected.size()) {
		 * autonomousSelected.get(currentAutoStep).Begin();
		 * }
		 * }
		 * } else {
		 * // stop drivetrain
		 * }
		 */

	}

	public void teleopInit() {

		holding = false;
		swerveDrive.zeroHeading();

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		// Controllers
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

	}

	public void teleopPeriodic() {

		if (operator.getRawButton(8)) {
			holding = false;
		}

		if (operator.getRawButtonPressed(5)) {
			if (holding) {
				holding = false;
			} else {
				holding = true;
			}
		}

		// cube
		if (holding) {
			motorGripper.set(0.5f);
		} else {
			if (operator.getRawButton(8)) {
				motorGripper.set(-0.2f);
			} else {
				motorGripper.set(0.0f);
			}
		}

		/*
		 * if (holding) {
		 * run motor
		 * } else {
		 * stop motor
		 * }
		 * 
		 * if (buttonPressed) {
		 * if (holding) {
		 * holding = false;
		 * } elese {
		 * holding = true;
		 * }
		 * }
		 */

		swerveDrive.drive(
				-MathUtil.applyDeadband(flightStickLeft.getRawAxis(0), OIConstants.kDriveDeadband),
				MathUtil.applyDeadband(flightStickLeft.getRawAxis(1), OIConstants.kDriveDeadband),
				-MathUtil.applyDeadband(flightStickRight.getRawAxis(0), OIConstants.kDriveDeadband),
				true, true);

		if (operator.getRawButton(4)) {

			solenoid1.set(Value.kReverse);
			solenoid2.set(Value.kForward);
		}

		if (operator.getRawButton(3)) {
			solenoid1.set(Value.kReverse);
			solenoid2.set(Value.kReverse);
		}

		if (operator.getRawButton(2)) {
			solenoid1.set(Value.kForward);
			solenoid2.set(Value.kReverse);

		}

		// cube close
		/*
		 * if (operator.getRawButton(7)) {
		 * motorGripper.set(0.2f);
		 * 
		 * } else if (operator.getRawButton(5)) {
		 * // cone close
		 * motorGripper.set(0.5f);
		 * 
		 * } else if (operator.getRawButton(8)) {
		 * motorGripper.set(-0.2f);
		 * 
		 * } else {
		 * motorGripper.set(0.0f);
		 * }
		 */

		// arm
		if (operator.getRawAxis(1) > 0.1f) {
			armExtend.set(-0.5f);
		} else if (operator.getRawAxis(1) < -0.1f) {
			armExtend.set(0.5f);
		} else {
			armExtend.set(0.0f);
		}
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
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void testPeriodic() {

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