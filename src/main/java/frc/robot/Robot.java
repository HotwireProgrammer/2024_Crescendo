package frc.robot;

import java.util.LinkedList;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autostep.AutoStep;
import frc.robot.swerve.Constants.OIConstants;
import frc.robot.swerve.DriveSubsystem;

public class Robot extends TimedRobot {

	// Joysticks
	public Joystick operator;
	public boolean arcadeDrive = false;
	public Joystick driver;

	// public CANSparkMax arm = new CANSparkMax(16, MotorType.kBrushless);

	public Limelight limelight = new Limelight();

	public DigitalInput beam = new DigitalInput(0);

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
   
	public float clawSpinOffset = 0;

	public DigitalInput clawStop = new DigitalInput(4);
	public Timer clawStopTimer = new Timer();
	public Timer shootWaitTimer = new Timer();


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
/*z
	public void autonomousInit() {
		currentAutoStep = 0;

	// first auto
		firstAuto = new LinkedList<AutoStep>();
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 1.2f));
		firstAuto.add(new MotorMoveStep(shooterWheelLower, 0.5f, 0.5f));
		firstAuto.add(new Wait(1.5f, swerveDrive));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, 0.25f, 0, 0, 0.4f));
		firstAuto.add(new MotorMoveStep(intake, 0.5f, -1.0f));
		firstAuto.add(new SwerveAutoDriveStep(swerveDrive, -0.25f, 0, 0, 1.0f));
		firstAuto.add(new Wait(1f, swerveDrive));
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
/*		twoNote = new LinkedList<AutoStep>();
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
*/
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

		System.out.println(beam.get());

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