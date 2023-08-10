package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class JoshMotorControllor {

	public PWMVictorSPX victor;
	public TalonSRX talon;
	public float accelValue;
	public float target;

	private boolean usingVictor;

	public JoshMotorControllor(int motorpwm, float AcelerationMax, boolean usingVictor) {
		if (usingVictor) {
			victor = new PWMVictorSPX(motorpwm);
		} else {
			talon = new TalonSRX(motorpwm);
		}

		accelValue = AcelerationMax;
		this.usingVictor = usingVictor;
	}

	public void UpdateMotor() {
		if (talon != null || victor != null) {

			double curr = 0;
			if (talon != null) {
				curr = talon.getMotorOutputPercent();
			}
			if (victor != null) {
				curr = victor.get();
			}

			float newValue = Lerp((float) curr, target, accelValue);

			float epsilon = 0.001f;
			if (newValue < epsilon && newValue > -epsilon) {
				newValue = 0.0f;
			}

			if (usingVictor) {
				victor.set(newValue);
			} else {
				talon.set(ControlMode.PercentOutput, target);
			}
		}
	}

	public float Lerp(float v0, float v1, float t) {
		return (v0 + t * (v1 - v0));
	}

	public void SetBrake() {
		if (talon != null) {
			talon.setNeutralMode(NeutralMode.Brake);
		}
	}

	public void SetCoast() {
		if (talon != null) {
			talon.setNeutralMode(NeutralMode.Coast);
		}
	}
}