package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LimelightDriver {
    // Min and max position in radians
    private static final double MAX_POSITION = 2.6;
    private static final double MIN_POSITION = -2.6;
    private static final double Kp = 0.025;
    private static final double Ki = 0;
    private static final double Kd = 0.0001;
    private final Limelight3A ll;
    private final Servo servo;
    private final IMU imu;
    private final Telemetry telemetry;
    // 0-centered, radians
    private double position = 0;
    private double heading = 0, oldHeading = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime memTrackTimer = new ElapsedTime();
    private double lastTime = 0;
    private double xIntegral = 0, xDerivative = 0, oldXError = 0;

    private enum LimelightState {
        LL_IDLE,
        LL_TRACK,
        LL_MEM
    }

    private LimelightState state;

    public LimelightDriver(
            Limelight3A limelight, Servo servo,
            IMU imu, Telemetry telemetry
    ) {
        this.ll = limelight;
        this.servo = servo;
        this.imu = imu;
        this.telemetry = telemetry;

        ll.pipelineSwitch(0);
        ll.start();

        servo.setPosition(0.5);
        loopTimer.reset();

        state = LimelightState.LL_MEM;
    }
    double filter(double a, double b) {
        return 0.2 * a + (1 - 0.2) * b;
    }

    double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    double memTx = 0;
    public void loop() {
        heading = normalize(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Heading", heading);

        LLResult res = ll.getLatestResult();

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        switch(state) {
            case LL_IDLE:
                if(res != null && res.isValid()) {
                    state = LimelightState.LL_TRACK; // We got a lock, start tracking
                }
                break;

            case LL_MEM:
                if(res != null && res.isValid()) {
                    state = LimelightState.LL_TRACK;
                    break;
                }

                memTx -= (oldHeading - heading); // Account for movement

                telemetry.addData("Memory track", "error %f, %f left",
                        memTx, 2 - memTrackTimer.seconds()
                );

                xIntegral += memTx * loopTime;

                xDerivative = filter((memTx - oldXError) / loopTime, xDerivative);

                double xUt_mem = Kp * memTx + Ki * xIntegral + Kd * xDerivative;

                oldXError = memTx;

                position += xUt_mem;
                memTx -= xUt_mem;

                if(position > MAX_POSITION)
                    position = MAX_POSITION;
                else if(position < MIN_POSITION)
                    position = MIN_POSITION;

                servo.setPosition((position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION));
                telemetry.addData("X error", memTx);
                telemetry.addData("PID out", xUt_mem);
                telemetry.addData("PID integral", xIntegral);
                telemetry.addData("PID derivative", xDerivative);
                break;

            case LL_TRACK:
                double xError;
                if(res != null && res.isValid()) {
                    xError = AngleUnit.DEGREES.toRadians(res.getTx());
                    memTx = xError;
                    memTrackTimer.reset();
                    telemetry.addLine("Tracking");
                } else {
                    if(memTrackTimer.seconds() > 0.1) {
                        memTrackTimer.reset();
                        // Lost lock, start memory track
                        state = LimelightState.LL_MEM;
                    }
                    break;
                }

                xIntegral += xError * loopTime;

                xDerivative = filter((xError - oldXError) / loopTime, xDerivative);

                double xUt = Kp * xError + Ki * xIntegral + Kd * xDerivative;

                oldXError = xError;

                position += xUt;

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

        oldHeading = heading;

        telemetry.addData("Position", position);
        telemetry.addData("Loop time", loopTime);
    }
}
