package org.firstinspires.ftc.teamcode.commands;

public abstract class Command {
    protected boolean done;
    public Command() {
        this.done = false;
    }

    public boolean isDone() {
        return done;
    }

    public abstract void run();
}