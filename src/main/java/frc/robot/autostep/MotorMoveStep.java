package frc.robot.autostep;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveTrain;
import com.revrobotics.CANSparkMax;

public class MotorMoveStep extends AutoStep {

    public Timer timer;
    public float time;
    public float speed;
    public CANSparkMax motor;

    public MotorMoveStep(CANSparkMax motor, float time, float speed) {
        super();
        this.time = time;
        this.speed = speed;
        this.motor = motor;
        timer = new Timer();
    }

    public void Begin() {
        timer.reset();
        timer.start();
    }

    public void Update() {
        motor.set(speed);
        if (timer.get() > time) {
            isDone = true;
            motor.set(0);
        }
    }
}