package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class LimelightDriver {
    private static final int FIDUCIALID_0 = 30;
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

    Pose pose = null;
    public void loop(double dx, double dy, double heading) {
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


                double targetAngle = Math.atan2(pose.y, pose.x);
                double currentAngle = heading + position;
                double memXError = currentAngle - targetAngle;

                telemetry.addData(
                        "Memory track", "%f %f, angle %f, current %f",
                        pose.x, pose.y, targetAngle, currentAngle
                );

                // Adjust for movement
                pose = new Pose(
                        pose.x - dx,
                        pose.y - dy,
                        0
                );

                xIntegral += memXError * loopTime;

                xDerivative = filter((memXError - oldXError) / loopTime, xDerivative);

                double xUt_mem = Kp * memXError + Ki * xIntegral + Kd * xDerivative;

                oldXError = memXError;

                position += xUt_mem;

                if(position > MAX_POSITION)
                    position = MAX_POSITION;
                else if(position < MIN_POSITION)
                    position = MIN_POSITION;

                servo.setPosition((position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION));
                telemetry.addData("X error", memXError);
                telemetry.addData("PID out", xUt_mem);
                telemetry.addData("PID integral", xIntegral);
                telemetry.addData("PID derivative", xDerivative);
                break;

            case LL_TRACK:
                if(res == null || !res.isValid()) {
                    if(memTrackTimer.seconds() > 0.1) {
                        memTrackTimer.reset();
                        // Lost lock, start memory track
                        state = LimelightState.LL_MEM;
                    }
                    break;
                }

                List<LLResultTypes.FiducialResult> fidRes = res.getFiducialResults();
                boolean found = false;
                double xError = 0;

                for(LLResultTypes.FiducialResult fid: fidRes) {
                    if(fid.getFiducialId() == FIDUCIALID_0) {
                        found = true;
                        Position pos = fid.getTargetPoseCameraSpace().getPosition();
                        pose = new Pose(pos.x, pos.y, 0);
                        xError = fid.getTargetXDegrees();
                        break;
                    }
                }

                if(!found) {
                    if(memTrackTimer.seconds() > 0.1) {
                        memTrackTimer.reset();
                        // Lost lock, start memory track
                        state = LimelightState.LL_MEM;
                    }
                    break;
                }

                memTrackTimer.reset();
                telemetry.addLine("Tracking");

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

        telemetry.addData("Position", position);
        telemetry.addData("Loop time", loopTime);
    }
}
