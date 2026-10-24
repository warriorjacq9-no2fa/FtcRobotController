package org.firstinspires.ftc.teamcode.turbo.commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Command;

public class ShootCommand extends Command {
    private enum ShootState {
        SPIN,
        SHOOTING,
        DONE
    }
    private ShootState state;
    private final DcMotorEx shooter;
    private final Servo gate;
    private final ElapsedTime shootTimer = new ElapsedTime();

    public ShootCommand(DcMotorEx shooter, Servo gate) {
        this.shooter = shooter;
        this.gate = gate;
        this.state = ShootState.SPIN;
    }
    @Override
    public void run() {
        switch(state) {
            case SPIN:
                shooter.setPower(0.7);
                shootTimer.reset();
                state = ShootState.SHOOTING;
                break;
            case SHOOTING:
                if(shootTimer.seconds() > 1.5) {
                    gate.setPosition(0.5);
                } else if(shootTimer.seconds() > 2) {
                    state = ShootState.DONE;
                }
                break;
            case DONE:
                done = true;
                break;
        }
    }
}
