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

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public CANSparkMax intake = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax claw = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax clawSpin = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax wench = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless);
	public Shooter shooter = new Shooter(limelight);
	public CANSparkMax elevator = new CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless);

	public RelativeEncoder clawSpinEncoder = clawSpin.getEncoder();
	public float clawSpinOffset = 0;

	public CANSparkMax climberOne = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
	public CANSparkMax climberTwo = new CANSparkMax(62, CANSparkLowLevel.MotorType.kBrushless);

	public DigitalInput limitSwitchOne = new DigitalInput(1);
	public DigitalInput limitSwitchTwo = new DigitalInput(0);
	public DigitalInput clawStop = new DigitalInput(6);
	public Timer clawStopTimer = new Timer();

	public boolean holding = false;

	public int climbStep = 0;
	public Timer climberStepTimer;
	public boolean firstClick = false;
	public Boolean secondClick = false;
	public boolean runShooter = false;

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {

		limelight.SetLight(false);
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

		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.0f));
		firstAuto.add(new Shoot(shooter, null, this, true));
		firstAuto.add(new Wait(1.0f, swerveDrive));
		firstAuto.add(new MotorMoveStep(claw, 1.0f, 0.5f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 0.25f));
		firstAuto.add(new MotorMoveStep(intake, 0.5f, -1.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 0.80f));
		firstAuto.add(new Wait(0.5f, swerveDrive));
		firstAuto.add(new MotorMoveStep(claw, 1.0f, 0.5f));

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
		if (runShooter) {
			shooter.Update(4900, 4200);
		}

	}

	public void teleopInit() {

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		// Controllers
		driver = new Joystick(1);
		operator = new Joystick(2);

		clawSpinEncoder.setPositionConversionFactor(1000);
		clawSpinOffset = (float) clawSpinEncoder.getPosition();
	}

	public void ClimberArmsUp(boolean up) {

		if (up) {

			if (firstClick) {
				climberOne.set(0.025);
			} else {
				climberOne.set(0.15);
			}

			if (limitSwitchOne.get()) {
				firstClick = true;
			}
		} else {
			climberOne.set(0.0);
		}

		if (up) {

			if (secondClick) {
				climberTwo.set(-0.025);
			} else {
				climberTwo.set(-0.15);
			}

			if (limitSwitchTwo.get()) {
				secondClick = true;
			}
		} else {
			climberTwo.set(0.0);
		}
	}

	public void teleopPeriodic() {

		boolean armsUp = false;
		if (operator.getRawButton(4)) {
			armsUp = true;
		}
		if (operator.getRawButtonPressed(4)) {
			firstClick = false;
			secondClick = false;
		}

		// shooter
		boolean clawRun = false;
		if (operator.getRawButton(6)) {

			if (operator.getRawButtonPressed(6)) {
				shooter.Reset();
			}

			shooter.Update(4900, 4200);

			if (operator.getRawButton(1)) {
				clawRun = true;
			}

		} else {
			shooter.PowerManual(0.0f);
		}

		if (operator.getRawButton(10)) {
			clawRun = true;
		}

		// driver

		// limelight climbing
		if (driver.getRawButton(2)) {

			// reset everything for a new climb
			if (driver.getRawButtonPressed(2)) {
				climbStep = 0;
				climberStepTimer = new Timer();
				climberStepTimer.reset();
			}

			/*
			 * Climb steps
			 * 8. shoot
			 */

			if (climbStep == 0) {
				// 1. position robot

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
					climberStepTimer.start();
				}

			} else if (climbStep == 1) {
				// 2. drive under chain

				swerveDrive.drive(0.1, 0, 0, false, true);

				if (climberStepTimer.get() > 0.8) {
					swerveDrive.drive(0, 0, 0, false, true);

					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}

			} else if (climbStep == 2) {
				// 3. Rotate claw
				clawSpin.set(0.25);

				if (climberStepTimer.get() > 0.5) {
					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}
			} else if (climbStep == 3) {
				// 4. back out
				swerveDrive.drive(-0.1, 0, 0, false, true);

				if (climberStepTimer.get() > 0.8) {
					swerveDrive.drive(0, 0, 0, false, true);

					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}
			} else if (climbStep == 4) {
				// 5. hook up

				armsUp = true;

				if (climberStepTimer.get() > 0.8) {
					swerveDrive.drive(0, 0, 0, false, true);

					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}

			} else if (climbStep == 5) {
				// 6. drive forward for hooks to connect

				swerveDrive.drive(0.1, 0, 0, false, true);

				if (climberStepTimer.get() > 0.8) {
					swerveDrive.drive(0, 0, 0, false, true);

					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}
			} else if (climbStep == 5) {
				// 7. wench up and elevator up

				wench.set(0.5);

				if (climberStepTimer.get() > 0.8) {
					swerveDrive.drive(0, 0, 0, false, true);

					climberStepTimer.reset();
					climbStep = climbStep + 1;
				}
			}

			// swerveDrive.drive(-0.05, 0.0, 0, false, true);
			// wench.set(1.0f);
		} else {

			// human controls
			if (operator.getRawAxis(3) > 0.01) {
				wench.set(operator.getRawAxis(3));
			} else if (operator.getRawAxis(2) > 0.01) {
				wench.set(-operator.getRawAxis(2));
			} else {
				wench.set(0);
			}

			// intake
			// System.out.println(clawStop.get());

			if (clawStop.get()) {
				clawStopTimer.reset();
				clawStopTimer.start();
			}
			if (operator.getRawButton(1) && clawStop.get() && clawStopTimer.get() < 0.5) {
				intake.set(-0.70);
				clawRun = true;
			} else {
				intake.set(0.0);
			}

			if (clawRun) {
				claw.set(0.5);
			} else {
				claw.set(0.0);
			}

			// claw
			{
				float idlePowerArm = 1.8f;
				float pi = 3.14159f;
				float armDegStart = 30f;
				float zeroOffsetDeg = -90.0f;

				float countPerHalfRotation = 2476;
				float countPerRotation = countPerHalfRotation * 2.0f;
				float degPerCount = 360.0f / countPerRotation;

				// arbitrary choice
				clawSpinEncoder.setPositionConversionFactor(1000);
				float armDeg = Math.abs(
						((float) Math.abs(clawSpinEncoder.getPosition() - clawSpinOffset) * degPerCount) + armDegStart
								- zeroOffsetDeg);
				float armRad = armDeg * (pi / 180.0f);
				double armPower = Math.cos(armRad);
				double gravityOffset = armPower * idlePowerArm;

				if (operator.getRawButton(5)) {
					clawSpin.setVoltage(gravityOffset + (operator.getRawAxis(5) * 1.5));
				} else {
					clawSpin.setVoltage(0);
				}

			}

			// elevator
			elevator.set(-operator.getRawAxis(1) * 0.20);

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

			if (driver.getRawButton(5)) {
				limelight.PositionRotate(swerveDrive);
			} else {

				swerveDrive.drive(
						MathUtil.applyDeadband(axisZero, OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(axisOne, OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(driver.getRawAxis(4), OIConstants.kDriveDeadband),
						true, true);
			}
		}

		// zero
		if (driver.getRawButton(1)) {
			swerveDrive.zeroHeading();
		}

		// set climber arms
		ClimberArmsUp(armsUp);
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