package org.firstinspires.ftc.teamcode.legacyturbo.commands;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Command;

import java.util.Locale;

public class ShootCommand extends Command {
    private static final double SHOOTER_SPEED = (1800.0 / 60) * 28;
    private enum ShootState {
        SPIN,
        SHOOTING,
        DONE
    }
    private ShootState state;
    private final DcMotorEx shooter;
    private final CRServo gate;
    private final ElapsedTime shootTimer = new ElapsedTime();

    public ShootCommand(DcMotorEx shooter, CRServo gate) {
        this.shooter = shooter;
        this.gate = gate;
        this.state = ShootState.SPIN;
    }
    @Override
    public void run() {
        switch(state) {
            case SPIN:
                shooter.setVelocity(SHOOTER_SPEED);
                shootTimer.reset();
                state = ShootState.SHOOTING;
                break;
            case SHOOTING:
                if(shootTimer.seconds() > 1.5) {
                    gate.setPower(1);
                }
                if(shootTimer.seconds() > 3) {
                    gate.setPower(0);
                    shooter.setVelocity(0);
                    state = ShootState.DONE;
                }
                break;
            case DONE:
                done = true;
                break;
        }
    }

    @NonNull
    public String toString() {
        return String.format(Locale.ENGLISH, "ShootCommand(%f): %s",
                this.shootTimer.seconds(), this.state.toString());
    }
}
