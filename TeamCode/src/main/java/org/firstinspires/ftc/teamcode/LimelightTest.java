package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name="LimelightTest")
public class LimelightTest extends OpMode {
    // Min and max position in radians
    private final double MAX_POSITION = 2.6;
    private final double MIN_POSITION = -2.6;
    private final double Kp = 0.025;
    private final double Ki = 0;
    private final double Kd = 0.0001;
    private final double SEARCH_SPEED = 0.5 * 2 * Math.PI; // Search speed in rad/s
    private Limelight3A ll;
    private Servo servo;
    // 0-centered, radians
    private double position;
    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime memTrackTimer = new ElapsedTime();
    private double lastTime;
    private double xIntegral, xDerivative, oldXError, oldXUt;

    private enum LimelightState {
        LL_SEARCH,
        LL_TRACK,
        LL_MEM
    }

    private LimelightState state;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "servo");

        ll.pipelineSwitch(0);
        ll.start();

        position = 0;
        servo.setPosition(0.5);
        loopTimer.reset();
        lastTime = 0;
        xIntegral = 0;
        xDerivative = 0;
        oldXError = 0;
        oldXUt = 0;

        state = LimelightState.LL_SEARCH;
    }
    double filter(double a, double b) {
        return 0.2 * a + (1 - 0.2) * b;
    }

    double searchDirection = 1;
    double memVel = 0;
    @Override
    public void loop() {
        LLResult res = ll.getLatestResult();
        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;
        switch(state) {
            case LL_SEARCH:
                if(res != null && res.isValid()) {
                    state = LimelightState.LL_TRACK; // We got a lock, start tracking
                    break;
                }

                position += SEARCH_SPEED * loopTime * searchDirection;
                if(position > MAX_POSITION) {
                    position = MAX_POSITION;
                    searchDirection = -1;
                } else if(position < MIN_POSITION) {
                    position = MIN_POSITION;
                    searchDirection = 1;
                }
                servo.setPosition((position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION));
                telemetry.addData("Searching", "%s", searchDirection == -1 ? "CW" : "CCW");
                break;

            case LL_MEM:
                if(res != null && res.isValid()) {
                    state = LimelightState.LL_TRACK; // We got a lock, start tracking
                    break;
                }
                
                position += memVel;

                if(position > MAX_POSITION)
                    position = MAX_POSITION;
                else if(position < MIN_POSITION)
                    position = MIN_POSITION;

                servo.setPosition((position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION));
                telemetry.addData("Memory track", "%f rad/s, %f left",
                        memVel, 2 - memTrackTimer.seconds()
                );

                if(memTrackTimer.seconds() >= 2) {
                    // We lost lock and are out of memory track, start searching
                    searchDirection = Math.signum(memVel);
                    if(searchDirection == 0) searchDirection = 1;
                    state = LimelightState.LL_SEARCH;
                }
                break;

            case LL_TRACK:
                double xError;
                if(res != null && res.isValid()) {
                    xError = AngleUnit.DEGREES.toRadians(res.getTx());
                    memTrackTimer.reset();
                    telemetry.addLine("Tracking");
                } else {
                    if(memTrackTimer.seconds() > 0.1) {
                        memTrackTimer.reset();
                        // Lost lock, start memory track
                        memVel = oldXUt; // Raw servo velocity
                        state = LimelightState.LL_MEM;
                    }
                    break;
                }

                xIntegral += xError * loopTime;

                xDerivative = filter((xError - oldXError) / loopTime, xDerivative);

                double xUt = Kp * xError + Ki * xIntegral + Kd * xDerivative;

                oldXError = xError;

                position += xUt;
                oldXUt = xUt;

                if(position > MAX_POSITION)
                    position = MAX_POSITION;
                else if(position < MIN_POSITION)
                    position = MIN_POSITION;

                servo.setPosition((position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION));
                telemetry.addData("X error", xError);
                telemetry.addData("PID out", xUt);
                telemetry.addData("PID integral", xIntegral);
                telemetry.addData("PID derivative", xDerivative);
        }
        telemetry.addData("Position", position);
        telemetry.addData("Loop time", loopTime);
        telemetry.update();
    }
}
