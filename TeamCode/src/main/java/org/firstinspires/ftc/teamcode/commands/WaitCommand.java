package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitCommand extends Command {
    private enum WaitState {
        INIT,
        RUN
    }

    private final double seconds;
    private WaitState state;
    private final ElapsedTime timer = new ElapsedTime();
    public WaitCommand(double seconds) {
        this.seconds = seconds;
    }
    @Override
    public void run() {
        switch(state) {
            case INIT:
                timer.reset();
                state = WaitState.RUN;
                break;

            case RUN:
                if(timer.seconds() == seconds)
                    done = true;
                break;
        }
    }
}
