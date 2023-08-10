package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain {
	JoshMotorControllor joshmotorcontrollorLeftBottomOne;
	JoshMotorControllor joshmotorcontrollorLeftBottomTwo;
	JoshMotorControllor joshmotorcontrollorRightBottomOne;
	JoshMotorControllor joshmotorcontrollorRightBottomTwo;
	float lerpSpeed = 1.0f;
	// public AHRS navx;

	public DriveTrain(int pwm1, int pwm2, int pwm3, int pwm4) {
		joshmotorcontrollorLeftBottomOne = new JoshMotorControllor(pwm1, lerpSpeed, false);
		joshmotorcontrollorLeftBottomTwo = new JoshMotorControllor(pwm2, lerpSpeed, false);
		// right bottom one

		joshmotorcontrollorRightBottomOne = new JoshMotorControllor(pwm3, lerpSpeed, false);
		joshmotorcontrollorRightBottomTwo = new JoshMotorControllor(pwm4, lerpSpeed, false);

		int limit = 240;
		int threshold = 240;
		float time = 0.001f;
		// joshmotorcontrollorLeftBottomOne.talon.configSupplyCurrentLimit(new
		// SupplyCurrentLimitConfiguration(false, limit, threshold, time));
		// joshmotorcontrollorLeftBottomTwo.talon.configSupplyCurrentLimit(new
		// SupplyCurrentLimitConfiguration(false, limit, threshold, time));
		// joshmotorcontrollorRightBottomOne.talon.configSupplyCurrentLimit(new
		// SupplyCurrentLimitConfiguration(false, limit, threshold, time));
		// joshmotorcontrollorRightBottomTwo.talon.configSupplyCurrentLimit(new
		// SupplyCurrentLimitConfiguration(false, limit, threshold, time));

		// this.navx = navx;
	}

	public void Update() {
		joshmotorcontrollorLeftBottomOne.UpdateMotor();
		joshmotorcontrollorLeftBottomTwo.UpdateMotor();
		joshmotorcontrollorRightBottomOne.UpdateMotor();
		joshmotorcontrollorRightBottomTwo.UpdateMotor();
	}

	public void SendData() {
		// SmartDashboard.putNumber("DriveTrain_LeftOne",
		// joshmotorcontrollorLeftBottomOne.talon.getTemperature());
		// SmartDashboard.putNumber("DriveTrain_LeftTwo",
		// joshmotorcontrollorLeftBottomTwo.talon.getTemperature());
		// SmartDashboard.putNumber("DriveTrain_RightOne",
		// joshmotorcontrollorRightBottomOne.talon.getTemperature());
		// SmartDashboard.putNumber("DriveTrain_RightTwo",
		// joshmotorcontrollorRightBottomTwo.talon.getTemperature());
	}

	public void SetLeftSpeed(float Speed) {
		joshmotorcontrollorLeftBottomOne.target = Speed;
		joshmotorcontrollorLeftBottomTwo.target = Speed;
	}

	public void SetRightSpeed(float Speed) {
		joshmotorcontrollorRightBottomOne.target = -Speed;
		joshmotorcontrollorRightBottomTwo.target = -Speed;
	}

	public void SetBothSpeed(float Speed) {
		SetLeftSpeed(Speed);
		SetRightSpeed(Speed);
	}

	public void SetBreak() {
		joshmotorcontrollorRightBottomOne.SetBrake();
		joshmotorcontrollorRightBottomTwo.SetBrake();
		joshmotorcontrollorLeftBottomOne.SetBrake();
		joshmotorcontrollorLeftBottomTwo.SetBrake();
	}

	public void SetCoast() {
		joshmotorcontrollorRightBottomOne.SetCoast();
		joshmotorcontrollorRightBottomTwo.SetCoast();
		joshmotorcontrollorLeftBottomOne.SetCoast();
		joshmotorcontrollorLeftBottomTwo.SetCoast();

	}

	public void DriveStraight(float speed, boolean reverse) {
		// float pidError = (float) turnController.get();
		float pidError = 0f;
		SetLeftSpeed((speed * pidError) + speed); // 0.6972
		SetRightSpeed(((speed) - (speed * pidError)) * -1); // -0.583

		speed = -speed;
		if (reverse) {
			speed = -speed;
		}
	}

	public double getEncoder() {
		return joshmotorcontrollorLeftBottomTwo.talon.getSelectedSensorPosition();
	}

	public double getEncoderLeft() {
		return joshmotorcontrollorLeftBottomOne.talon.getSelectedSensorPosition();
	}

	public double getEncoderRight() {
		return joshmotorcontrollorRightBottomTwo.talon.getSelectedSensorPosition();
	}

	public double getEncoderSpeed() {
		return joshmotorcontrollorLeftBottomTwo.talon.getSelectedSensorVelocity();
	}
}