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
import edu.wpi.first.networktables.BooleanEntry;
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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

	// Joysticks
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick driver;

	// public CANSparkMax arm = new CANSparkMax(16, MotorType.kBrushless);

	public Limelight limelight = new Limelight();

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriveSubsystem swerveDrive = new DriveSubsystem();

	// Auto
	public LinkedList<AutoStep> firstAuto;
	public LinkedList<AutoStep> autoStraight;
	public LinkedList<AutoStep> twoNote;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	// cansparkmax example
    public CANSparkMax intake = new CANSparkMax(7, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax intakeWheelLeft = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax intakeWheelRight = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax shooterWheelUpper = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax shooterWheelLower = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax feederBeltLeft = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax feederBeltRight = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax shooterRotationTop = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax shooterRotationBottom = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless);

	public float clawSpinOffset = 0;

	public DigitalInput limitSwitchOne = new DigitalInput(1);
	public DigitalInput limitSwitchTwo = new DigitalInput(0);
	public DigitalInput clawStop = new DigitalInput(4);
	public Timer clawStopTimer = new Timer();

	public boolean holding = false;

	public int climbStep = 0;
	public Timer climberStepTimer;
	public boolean firstClick = false;
	public Boolean secondClick = false;
	public boolean runShooter = false;
	public boolean firstClaw = false;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {

		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		swerveDrive.Init();
	}

	public void disabledInit() {

		// Controllers
		operator = new Joystick(2);
		driver = new Joystick(1);
	}

	public void disabledPeriodic() {

	}

	public void autonomousInit() {
		currentAutoStep = 0;

	// first auto
		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.2f));
//		firstAuto.add(new Shoot(shooter, null, this, true));
		firstAuto.add(new Wait(1.5f, swerveDrive));
//		firstAuto.add(new MotorMoveStep(claw, 1.0f, 0.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 0.4f));
		firstAuto.add(new MotorMoveStep(intake, 0.5f, -1.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 1.0f));
		firstAuto.add(new Wait(1f, swerveDrive));
//		firstAuto.add(new MotorMoveStep(claw, 1.0f, 0.5f));
		// After first two shots
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 0.3f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 90, 0.25f, 5));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0f, -0.25f, 0, 0.65f));
		firstAuto.add(new MotorMoveStep(intake, 1.0f, -1.0f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 0, 0.25f, 5));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0f, 0.25f, 0, 1.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 0.4f));
		firstAuto.add(new NavxTurn(swerveDrive, swerveDrive.m_gyro, 0, 0.25f, 5));
		firstAuto.add(new Wait(1f, swerveDrive));
//		firstAuto.add(new MotorMoveStep(claw, 1.0f, 0.5f));
//		firstAuto.add(new Shoot(shooter, null, this, false));


		// auto straight
		autoStraight = new LinkedList<AutoStep>();
		autoStraight.add(new Wait(10f, swerveDrive));
		autoStraight.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 4.0f));

		// two note
		twoNote = new LinkedList<AutoStep>();
		twoNote.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.2f));
//		twoNote.add(new Shoot(shooter, null, this, true));
		twoNote.add(new Wait(1.5f, swerveDrive));
//		twoNote.add(new MotorMoveStep(claw, 1.0f, 0.5f));
		twoNote.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 0.4f));
		twoNote.add(new MotorMoveStep(intake, 0.5f, -1.0f));
		twoNote.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 1.0f));
		twoNote.add(new Wait(1f, swerveDrive));
//		twoNote.add(new MotorMoveStep(claw, 1.0f, 0.5f));

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);

		autonomousSelected = firstAuto;
		if (autoChoice == 0) {
			autonomousSelected = firstAuto;
		} else if (autoChoice == 1) {
			autonomousSelected = autoStraight;
		} else if (autoChoice == 2) {
			autonomousSelected = twoNote;
		}

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

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		// Controllers
		driver = new Joystick(1);
		operator = new Joystick(2);

		
	}


	public void teleopPeriodic() {

		// drive controls
		double pow = 2;
		double axisZero = -Math.pow(driver.getRawAxis(1), pow)
				* (driver.getRawAxis(1) / Math.abs(driver.getRawAxis(1)));
		double axisOne = Math.pow(driver.getRawAxis(0), pow)
				* (driver.getRawAxis(0) / Math.abs(driver.getRawAxis(0)));

		if (driver.getRawButton(6)) {
			axisZero = axisZero * 0.25;
			axisOne = axisOne * 0.25;
		}

		if (driver.getRawAxis(3) > 0.1) {
			swerveDrive.drive(
					MathUtil.applyDeadband(axisZero, OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(axisOne, OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(driver.getRawAxis(4), OIConstants.kDriveDeadband),
					false, true);
		} else {
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

		if (operator.getRawButton(6)) {
			shooterWheelUpper.set(0.80);
			shooterWheelLower.set(0.80);
			
		} else {
			shooterWheelUpper.set(0.0);
			shooterWheelLower.set(0.0);
		}

		if (operator.getRawButton(5)) {
			feederBeltLeft.set(0.80);
			feederBeltRight.set(-0.80);
		} else {
			feederBeltLeft.set(0.0);
			feederBeltRight.set(0.0);
		}

		if (operator.getRawButton(1)){
			intake.set(-0.80);
			intakeWheelLeft.set(.50);
			intakeWheelRight.set(-0.50);
		} else {
			intake.set((0.0));
			intakeWheelLeft.set(0.0);
			intakeWheelRight.set(0.0);
		}

		if (driver.getRawButton(1)) {
			shooterRotationTop.set(-0.5);
			shooterRotationBottom.set(-0.5);
		} else {
			shooterRotationTop.set(0.0);
			shooterRotationBottom.set(0.0);
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
		operator = new Joystick(2);
		driver = new Joystick(1);

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