package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter {
    public CANSparkMax shooterFeeder = new CANSparkMax(7, MotorType.kBrushless);
    public CANSparkMax shooterBottom = new CANSparkMax(4, MotorType.kBrushless);
    public CANSparkMax shooterTop = new CANSparkMax(5, MotorType.kBrushless);

    public Limelight limelight;

    public double shooterP = 0.0002;
    public double shooterI = 0.001;
    public double shooterD = 0.0;

    public HotPID topPid = new HotPID("shooter top", shooterP, shooterI, shooterD);
    public HotPID bottomPid = new HotPID("shooter bottom", shooterP, shooterI, shooterD);

    public double rpmCurrent = 0;

    private double rpmTarget = 0;

    public Shooter(Limelight limelight) {

        this.limelight = limelight;
    }

    public void Init() {
        topPid = new HotPID("shooter top", shooterP, shooterI, shooterD);
        bottomPid = new HotPID("shooter bottom", shooterP, shooterI, shooterD);

    }

    public void Reset() {
        topPid.reset();
        bottomPid.reset();
    }

    public void Update(float rpmTop, float rpmBottom) {

        // shooterTop.get
        SmartDashboard.putNumber("shooter rpm Top", shooterTop.getEncoder().getVelocity());
        SmartDashboard.putNumber("shooter rpm Bottom", shooterBottom.getEncoder().getVelocity());

        topPid.setpoint = rpmTop;
        double motorSpeedTop = topPid.Calculate(shooterTop.getEncoder().getVelocity());

        bottomPid.setpoint = rpmBottom;
        double motorSpeedBottom = bottomPid.Calculate(shooterBottom.getEncoder().getVelocity());

        if (motorSpeedTop < 0) {
            motorSpeedTop = 0;
        }
        if (motorSpeedBottom < 0) {
            motorSpeedBottom = 0;
        }

        if (rpmTop == 0.0) {
            PowerManual(0);
        } else {
            shooterTop.set(motorSpeedTop);
            shooterBottom.set(motorSpeedBottom);
            shooterFeeder.set(-1);
        }
    }

    public boolean UpToSpeed(float RPMBuffer) {
        double distance = Math.abs(rpmCurrent - rpmTarget);
        return distance < (rpmTarget * RPMBuffer);
    }

    public void PowerManual(float power) {
        shooterTop.set(power);
        shooterBottom.set(power * -1);
        shooterFeeder.set(0);
    }

    public float Lerp(float v0, float v1, float t) {

        if (t <= 0) {
            t = 0;
        } else if (t >= 1) {
            t = 1;
        }

        return (v0 + t * (v1 - v0));
    }
}
