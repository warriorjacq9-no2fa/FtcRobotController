package org.firstinspires.ftc.teamcode.mechbots.commands;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private final DcMotorEx sGate;
    private final ElapsedTime shootTimer = new ElapsedTime();

    public ShootCommand(DcMotorEx shooter, DcMotorEx sGate) {
        this.shooter = shooter;
        this.sGate = sGate;
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
                if(shootTimer.seconds() > 1) {
                    sGate.setPower(0.5);
                } else if(shootTimer.seconds() > 1.5) {
                    state = ShootState.DONE;
                }
                break;
            case DONE:
                done = true;
                break;
        }
    }
}
