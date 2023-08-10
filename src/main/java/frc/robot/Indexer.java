package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import javax.lang.model.util.ElementScanner6;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Indexer {
    public int ballCountD;
    public int ballCount = 0;
    public boolean firstShot = true;

    public boolean firstInitialTrigger = false;

    public DigitalInput firstBeam   = new DigitalInput(1);
    public DigitalInput secondBeam = new DigitalInput(0);

    public TalonSRX indexerMotor = new TalonSRX(9);

    private Shooter shooter;
    private PreShooterpid preShooter;

    boolean toggleencoder = false;
    boolean didPause = true;
    boolean shooting = false;
    Timer timer = new Timer();
    boolean toggle2 = false;

    private float pauseTime = 0.2f;

    public Indexer(Shooter shooter, PreShooterpid preShooter) {
        this.shooter = shooter;
        this.preShooter = preShooter;
    }
    public void resetBallCountD(){
        ballCountD = 0;
    }
    public void RunManualForward(float speed, float RPMBuffer) {
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        boolean upToSpeed = shooter.UpToSpeed(RPMBuffer) && preShooter.UpToSpeed(RPMBuffer * 1.5f);
        ballCount = 0;
        // firstShot = true;
        if (!secondBeam.get()){
            timer.reset();
            timer.start();
        }
        if (secondBeam.get()){
            toggle2 = true;
        }else if (toggle2 && !secondBeam.get() && timer.hasElapsed(0.1)){
            firstShot = false;
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
        if (firstShot && upToSpeed){
            indexerMotor.set(ControlMode.PercentOutput, -speed);
        }else if (secondBeam.get() && timer.hasElapsed(0.15) &&upToSpeed){
            indexerMotor.set(ControlMode.PercentOutput, -speed);
            firstShot = true;
        }else{
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
        // } else {
        //     indexerMotor.set(ControlMode.PercentOutput, 0);
        // }
        //System.out.println(speed);
    }

    public void RunAutomatic() {
        indexerMotor.setNeutralMode(NeutralMode.Brake);
        SmartDashboard.putNumber("ball counter", ballCountD);
        toggleencoder = false;
        if (secondBeam.get()) {
            float targetSpeed = 0;

            if (ballCount == 0) {
                if (!firstBeam.get()) {
                    firstInitialTrigger = true;
                    ballCountD = 1;
                }

                if (firstInitialTrigger && firstBeam.get()) {
                    ballCount = 1;
                }

                targetSpeed = 0.4f;
            }

            if (ballCount == 1) {
                if (!firstBeam.get() && secondBeam.get()) {
                    ballCountD = 2;
                    targetSpeed = 0.5f;
                }
            }
            //System.out.println(targetSpeed + " target speed");
            indexerMotor.set(ControlMode.PercentOutput, -targetSpeed);
        } else {
            indexerMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void DebugPrint() {
        // System.out.println(" 1-" + beamBreakOne.get() + " top-" +
        // beamBreakTop.get());
    }
}