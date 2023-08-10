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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.DriverStation;

import java.applet.AudioClip;
import java.nio.Buffer;
import java.rmi.server.Operation;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.autostep.*;
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

	// Sensors
	public AnalogGyro headinGryo = new AnalogGyro(0);
	public static AHRS sensorNavx = new AHRS();

	// Drive train
	public DriveTrain driveTrain = new DriveTrain(1, 2, 3, 4);
	// public TalonSRX frontLeft = new TalonSRX(1);
	// public TalonSRX backLeft = new TalonSRX(2);
	// public TalonSRX frontRight = new TalonSRX(3);
	// public TalonSRX backRight = new TalonSRX(4);

	// Logic
	public float pwrm = 1;
	public double setPointArm = 0;

	// ArmLogic
	public boolean cone = false;

	// Robot Components
	public Gripper gripper;
	public Arm arm = new Arm();

	// Joysticks
	// public Joystick driver;
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick flightStickLeft;
	public Joystick flightStickRight;

	public Limelight limelight = new Limelight();

	public HotPID navxPID = new HotPID("navx", 0.01, 0.01, 0.0);// was 0.0005
	public HotPID pidAutoBalance = new HotPID("Balance", -0.05, 0, 0);

	public enum DriveScale {
		linear, squared, tangent, inverse, cb, cbrt,
	}

	public DriverStation driverStation;
	public RobotState currentState;

	// Auto
	public LinkedList<AutoStep> firstAuto;
	public LinkedList<AutoStep> secondAuto;
	public LinkedList<AutoStep> testAuto;
	public LinkedList<AutoStep> test2Auto;

	public LinkedList<AutoStep> autonomousSelected;
	public int currentAutoStep = 0;

	public String autoSelectKey = "autoMode";

	public float voltComp(float percent) {
		return (float) (12.6 * percent / RobotController.getBatteryVoltage());
	}

	public void robotInit() {
		arm.ResetEncoder();
		SmartDashboard.putNumber("Ballcount", 0);
		SmartDashboard.putBoolean("Balance", false);
		SmartDashboard.putBoolean("DriveStraight", false);
		SmartDashboard.putBoolean("Test", false);
		CameraServer.startAutomaticCapture();
		limelight.SetLight(false);
		limelight.Init();
		SmartDashboard.putNumber(autoSelectKey, 0);

		gripper = new Gripper(this, arm);
	}

	public void periodic() {
		var heading = Rotation2d.fromDegrees(headinGryo.getAngle());
	}

	public void disabledInit() {

		// Controllers
		driveTrain.SetBreak();
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public void disabledPeriodic() {
		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", false);
	}

	public void autonomousInit() {
		arm.setPointArm = 1.57f;
		arm.powerBool = false;
		currentAutoStep = 0;
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);

		operator = new Joystick(1);

		driveTrain.SetBreak();
		limelight.SetLight(true);

		// Autonomous selection

		double autoChoice = SmartDashboard.getNumber(autoSelectKey, 0);
		boolean test = SmartDashboard.getBoolean("Test", false);
		boolean balance = SmartDashboard.getBoolean("Balance", false);
		boolean driveStraight = SmartDashboard.getBoolean("DriveStraight", false);


		// firstAuto.add(new TimedForward(driveTrain, 1, 0.2f));
		firstAuto = new LinkedList<AutoStep>();

		firstAuto.add(new NavxReset(sensorNavx));
		firstAuto.add(new GripperStep(gripper, true));
		firstAuto.add(new ArmZero(arm));
		firstAuto.add(new ArmMove(arm, 0.7f, -1.0f, operator));
		firstAuto.add(new ArmPushPull(arm, 0.5f, true));
		firstAuto.add(new TimedForward(driveTrain, 0.7f, -0.2f));
		firstAuto.add(new MotorMoveStep(gripper.motorGripper, 0.3f, -0.3f));
		firstAuto.add(new ArmPushPull(arm, 0.5f, false));
		firstAuto.add(new EncoderForwardFeet(driveTrain, 1.0f, 0.4f));
		firstAuto.add(new ArmPower0(arm, true));
		firstAuto.add(new EncoderForwardFeet(driveTrain, 4.0f, 0.4f));

		firstAuto.add(new NavxDriveUntil(sensorNavx, 5, 0.2f, driveTrain));


		firstAuto.add(new EncoderForwardFeet(driveTrain, 3.0f, 0.4f));
		firstAuto.add(new Wait(driveTrain, 2.0f));
		firstAuto.add(new EncoderForwardFeet(driveTrain, 4.0f, -0.4f));



		firstAuto.add(new NavxPIDLevel(sensorNavx, driveTrain));

		firstAuto.add(new ArmPower0(arm, false));

		secondAuto = new LinkedList<AutoStep>();

		secondAuto.add(new NavxReset(sensorNavx));
		secondAuto.add(new GripperStep(gripper, true));
		secondAuto.add(new ArmZero(arm));
		secondAuto.add(new ArmMove(arm, 1.5f, -0.5f, operator));
		secondAuto.add(new ArmPushPull(arm, 1.2f, true));
		secondAuto.add(new TimedForward(driveTrain, 0.7f, -0.2f));
		secondAuto.add(new MotorMoveStep(gripper.motorGripper, 0.3f, -0.3f));
		secondAuto.add(new ArmPushPull(arm, 0.5f, false));
		secondAuto.add(new EncoderForwardFeet(driveTrain, 1.0f, 0.4f));
		secondAuto.add(new ArmPower0(arm, true));
		secondAuto.add(new EncoderForwardFeet(driveTrain, 3.0f, 0.4f));
		secondAuto.add(new EncoderForwardFeet(driveTrain, 1.0f, 0.2f));

		secondAuto.add(new EncoderForwardFeet(driveTrain, 9.0f, 0.4f));

		testAuto = new LinkedList<AutoStep>();

		testAuto.add(new NavxReset(sensorNavx));

		// testAuto.add(new EncoderForwardFeet(driveTrain, 4.0f, -0.6f));
		testAuto.add(new NavxPIDLevel(sensorNavx, driveTrain));

		test2Auto = new LinkedList<AutoStep>();

		test2Auto.add(new NavxReset(sensorNavx));
		test2Auto.add(new GripperStep(gripper, true));
		test2Auto.add(new ArmZero(arm));
		test2Auto.add(new ArmMove(arm, 1.5f, -0.5f, operator));
		test2Auto.add(new ArmPushPull(arm, 1.2f, true));
		// test2Auto.add(new TimedForward(driveTrain, 0.7f, -0.2f));
		test2Auto.add(new MotorMoveStep(gripper.motorGripper, 0.3f, -0.3f));
		test2Auto.add(new ArmPushPull(arm, 0.5f, false));
		test2Auto.add(new EncoderForwardFeet(driveTrain, 1.0f, 0.4f));
		test2Auto.add(new ArmPower0(arm, true));
		test2Auto.add(new EncoderForwardFeet(driveTrain, 3.0f, 0.4f));
		test2Auto.add(new EncoderForwardFeet(driveTrain, 1.0f, 0.2f));

		test2Auto.add(new NavxDriveUntil(sensorNavx, 5, 0.2f, driveTrain));

		// test2Auto.add(new TimedForward(driveTrain, 1.0f, 0.3f));
		test2Auto.add(new Wait(driveTrain, 1.0f));
		// test2Auto.add(new TimedForward(driveTrain, 1.0f, -0.4f));

		// test2Auto.add(new NavxPIDLevel(sensorNavx, driveTrain));

		test2Auto.add(new ArmPower0(arm, false));

		// firstAuto.add(new NavxDriveUntil(sensorNavx, 5, 0.2f, driveTrain));
		// firstAuto.add(new TimedForward(driveTrain, 1.0f, 0.3f));
		// firstAuto.add(new Wait(driveTrain, 0.25f));
		// firstAuto.add(new TimedForward(driveTrain, 1.0f, -0.3f));
		// firstAuto.add(new NavxPIDLevel(sensorNavx, driveTrain));
		// firstAuto.add(new ArmPower0(arm, false));

		// firstAuto.add(new ArmPushPull(arm, 0.5f, false));

		// firstAuto.add(new ArmMove(arm, 1.5f, -0.1f, operator));

		// firstAuto.add(new MotorMoveStep(gripper.motorGripper, 0.3f, 0.5f));

		// firstAuto.add(new ArmMove(arm, 1.0f, 0.1f, operator));
		// firstAuto.add(new TimedForward(driveTrain, 1.5f, -0.5f));
		// firstAuto.add(new TimedTurn(driveTrain, 0.4f, -0.6f));

		if (autoChoice == 0) {
			autonomousSelected = firstAuto;
		} else if (autoChoice == 1) {
			autonomousSelected = secondAuto;
		} else if (autoChoice == 2) {
			autonomousSelected = testAuto;
		} else if (autoChoice == 3) {
			autonomousSelected = test2Auto;
		} else {
			autonomousSelected = testAuto;
		}
		// autonomousSelected = test2Auto;
		if(balance){
			autonomousSelected = firstAuto;
		}
		if(driveStraight){
			autonomousSelected = secondAuto;
		}

		// autonomousSelected = firstAuto;
		autonomousSelected.get(0).Begin();
	}

	public void autonomousPeriodic() {
		SmartDashboard.putBoolean("RobotEnabled", true);

		arm.Update(0, operator);
		arm.AutoUpdate();
		gripper.AutoPeriodic();

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
			driveTrain.SetBothSpeed(0.0f);
			// currentState = RobotState.Teleop;
		}

		UpdateMotors();
	}

	public void teleopInit() {
		arm.setPointArm = 1.57f;
		arm.powerBool = false;

		sensorNavx.reset();

		limelight.SetLight(false);

		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

		driveTrain.SetCoast();

		// Controllers
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);

	}

	// Drive Scale
	boolean slow = false;

	public float DriveScaleSelector(float ControllerInput, DriveScale selection) {

		float multiplier = (ControllerInput / (float) Math.abs(ControllerInput));

		if (selection == DriveScale.squared) {
			float output = multiplier * (float) (ControllerInput * ControllerInput);

			if (flightStickLeft.getRawButtonPressed(1)) {
				slow = !slow;
			}

			if (slow) {
				output = output * 0.5f;
			}

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

	public void teleopPeriodic() {

		// System.out.println(sensorNavx.getPitch());

		if (operator.getRawButton(10)) {
			arm.ResetEncoder();
		}

		arm.OffsetGravity(cone, false);

		gripper.teleopPeriodic();

		{
			float speed = 1.0f;

			if (flightStickLeft.getRawButtonPressed(1)) {
				setPointArm = setPointArm + 0.1;
			} else if (flightStickRight.getRawButtonPressed(1)) {
				setPointArm = setPointArm - 0.1;
			}
			// System.out.println(operator.getRawAxis(1));
			arm.Update(operator.getRawAxis(3), operator);
			arm.debug();
			if (operator.getRawButton(9)) {
				arm.PowerManual(0);
			}

			if (operator.getRawButton(8)) {
				driveTrain.SetBreak();
			} else {
				driveTrain.SetCoast();
			}
			ControllerDrive();
			driveTrain.Update();
		}

		// System.out.println(frontLeft.getSelectedSensorVelocity() + " front left " +
		// backLeft.getSelectedSensorVelocity()
		// + " back left " + frontRight.getSelectedSensorVelocity() + " front right "
		// + backRight.getSelectedSensorVelocity() + " back right");

		pwrm = 12 / (float) RobotController.getBatteryVoltage();
		float rampTime = 0.0f;
		// frontLeft.configOpenloopRamp(rampTime);
		// backLeft.configOpenloopRamp(rampTime);
		// frontRight.configOpenloopRamp(rampTime);
		// backRight.configOpenloopRamp(rampTime);

		driveTrain.SendData();
		SmartDashboard.putBoolean("RobotEnabled", true);

		// Lime Light
		if (flightStickLeft.getRawButton(3)) {
			limelight.Position(driveTrain);
			driveTrain.SetBreak();
		} else {
			// driveTrain.SetCoast();
			// ControllerDrive();
		}

		// if (flightStickRight.getRawButton(1)) {
		// 	// auto balance

		// 	float error = -sensorNavx.getPitch();
		// 	double output = pidAutoBalance.Calculate(error);
		// 	driveTrain.SetBothSpeed((float) output);
		// } else {
			ControllerDrive();
		

		UpdateMotors();
	}

	public void testInit() {

		// navx.reset();
		// climber.coastMode();

		// Controllers
		// driver = new Joystick(0);
		operator = new Joystick(1);
		flightStickLeft = new Joystick(3);
		flightStickRight = new Joystick(2);
	}

	public static float Lerp(float v0, float v1, float t) {

		if (t < 0) {
			t = 0;

		} else if (t > 1) {
			t = 1;
		}

		return (v0 + t * (v1 - v0));
	}

	// ffloat navxTarget;
	// NavxTurnPID testTurn = new NavxTurnPID(driveTrain, navx, 10, 2.5f, navxPID);

	// DigitalInput beamTest = new DigitalInput(1);
	// beamTest.get();
	// intakeSeven.set(ControlMode.PercentOutput, 0.5f);
	// flightStickLeft.getRawButtonPressed(0);

	/*
	 * - run intake when flight stick top button pressed
	 * - unless the ball is in beam break then don't run the intake
	 * - run intake backwards always, regardless of ball when trigger button is
	 * pressed
	 */
	// DigitalInput beamTest = new DigitalInput(1);

	public void testPeriodic() {


		double volts = operator.getRawAxis(0) * 5.0;
		arm.PowerManual(0.0f);
		arm.ResetEncoder();

		driveTrain.SetBothSpeed(0.0f);
		driveTrain.SetCoast();
		driveTrain.Update();
		UpdateMotors();
	}

	public void ControllerDrive() {
		if (arcadeDrive) {

			// Arcade
			// float horJoystick = TranslateController((float) driver.getRawAxis(2));
			// float verJoystick = TranslateController((float) driver.getRawAxis(1));

			// float horJoystick = DriveScaleSelector((float) driver.getRawAxis(2),
			// DriveScale.parabala);
			// float verJoystick = DriveScaleSelector((float) driver.getRawAxis(1),
			// DriveScale.parabala);

			// driveTrain.SetRightSpeed(-verJoystick + -horJoystick);
			// driveTrain.SetLeftSpeed(-verJoystick + horJoystick);
			// driveTrain.SetCoast();
		} else {
			// tank
			// float leftJoystick = DriveScaleSelector((float)
			// flightStickLeft.getRawAxis(1), DriveScale.parabala);
			// float rightJoystick = (DriveScaleSelector((float)
			// flightStickRight.getRawAxis(1), DriveScale.parabala));
			// driveTrain.SetRightSpeed((-rightJoystick));
			// driveTrain.SetLeftSpeed((-leftJoystick));

			float leftJoystick = DriveScaleSelector((float) flightStickLeft.getRawAxis(1), DriveScale.linear);
			float rightJoystick = (DriveScaleSelector((float) flightStickRight.getRawAxis(0), DriveScale.linear));
			float expo = 0.4f;
			rightJoystick = (float) (expo * Math.pow(rightJoystick, 3) + (1 - expo) * rightJoystick);
			// System.out.println(rightJoystick+ " right joystick");

			driveTrain.SetRightSpeed(-leftJoystick + -rightJoystick);
			driveTrain.SetLeftSpeed(-leftJoystick + rightJoystick);

			boolean overrideAntiTip = false;
			if (flightStickLeft.getRawButton(1)) {
				overrideAntiTip = true;
			}
			if ((Math.abs(sensorNavx.getPitch()) > 5.0f) && (!overrideAntiTip) && (Math
					.abs((sensorNavx.getRawGyroX() - sensorNavx.getPitch())) < Math.abs(sensorNavx.getPitch()))) {
				// driveTrain.SetBothSpeed(sensorNavx.getPitch()*0.05f);
			}
			driveTrain.Update();

			// driveTrain.SetRightSpeed(leftJoystick);
			// driveTrain.SetLeftSpeed(rightJoystick);
			// driveTrain.SetCoast();

		}
	}

	public void UpdateMotors() {
		driveTrain.Update();
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