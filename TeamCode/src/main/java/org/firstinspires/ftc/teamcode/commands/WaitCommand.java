package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

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
        this.state = WaitState.INIT;
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

    @NonNull
    public String toString() {
        return String.format(Locale.ENGLISH, "WaitCommand(%f): %s",
                this.seconds, this.state.toString());
    }
}
