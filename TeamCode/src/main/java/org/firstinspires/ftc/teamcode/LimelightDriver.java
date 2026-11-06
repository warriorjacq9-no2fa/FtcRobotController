package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@Config
public class LimelightDriver {
    /* Config variables */
    public static int FIDUCIAL_ID_0 = 30;
    /* Min and max position in radians */
    public static double MAX_POSITION = 2.6;
    public static double MIN_POSITION = -2.6;
    public static PIDCoefficients pid = new PIDCoefficients(0.025, 0, 0.0001);

    /* Instance config variables */
    public double ENCODER_CPR = 537.7;
    public double TURRET_GEAR_RATIO = 1 / 2.75;
    /*
     * Speed of turret projectile in meters per second
     * Used for calculating lead angle
     * Set to 0 to disable
     */

    public double PROJECTILE_SPEED = 0;

    private final Limelight3A ll;
    private final DcMotorEx turret;
    private final Driver driver;
    private final Telemetry telemetry;
    // 0-centered, radians
    private double position = 0;
    private double oldEncoder = 0;
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
            Limelight3A limelight, DcMotorEx turret,
            Driver driver, Telemetry telemetry
    ) {
        this.ll = limelight;
        this.turret = turret;
        this.driver = driver;
        this.telemetry = telemetry;

        ll.pipelineSwitch(0);
        ll.start();

        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

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

    double getPerpendicularVelocity(Pose targetPose, Pose velocity) {
        /* Get distance to target */
        double magnitude = Math.hypot(targetPose.x, targetPose.y);
        if(magnitude < 1e-6) return 0;

        /*
         * Create a unit vector from targetPose
         * A unit vector has a distance of 1
         */
        double ux = targetPose.x / magnitude;
        double uy = targetPose.y / magnitude;

        /*
         * Use the cross product to get velocity
         * perpendicular to the target
        */
        return velocity.y * ux - velocity.x * uy;
    }

    Pose oldPose = new Pose(0, 0, 0);
    Pose pose = new Pose(0, 0, 0);
    public void loop() {
        LLResult res = ll.getLatestResult();

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        double encoder = turret.getCurrentPosition();
        position += ((encoder - oldEncoder) / ENCODER_CPR * TURRET_GEAR_RATIO) * 2 * Math.PI;
        position = normalize(position);
        oldEncoder = encoder;

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

                Pose currentPose = driver.getCurrentPose();
                Pose currentVelocity = driver.getCurrentVelocity();

                double curVelTargetSpace = getPerpendicularVelocity(pose, currentVelocity);

                double targetAngle = Math.atan2(pose.y, pose.x);
                if(PROJECTILE_SPEED > 0) targetAngle += Math.asin(curVelTargetSpace / PROJECTILE_SPEED);
                double currentAngle = normalize(currentPose.heading + position);
                double memXError = currentAngle - targetAngle;

                telemetry.addData("Memory track target X", pose.x);
                telemetry.addData("Memory track target Y", pose.y);
                telemetry.addData("Memory track target angle", targetAngle);
                telemetry.addData("Memory track current angle", currentAngle);

                double dx = currentPose.x - oldPose.x;
                double dy = currentPose.y - oldPose.y;

                // Adjust for movement
                pose = new Pose(
                        pose.x - dx,
                        pose.y - dy,
                        0
                );

                xIntegral += memXError * loopTime;

                xDerivative = filter((memXError - oldXError) / loopTime, xDerivative);

                double xUt_mem = pid.p * memXError + pid.i * xIntegral + pid.d * xDerivative;

                oldXError = memXError;

                turret.setPower(xUt_mem);

                telemetry.addData("Angle error", memXError);
                telemetry.addData("PID out", xUt_mem);
                telemetry.addData("PID integral", xIntegral);
                telemetry.addData("PID derivative", xDerivative);

                oldPose = currentPose;
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
                    if(fid.getFiducialId() == FIDUCIAL_ID_0) {
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

                double xUt = pid.p * xError + pid.i * xIntegral + pid.d * xDerivative;

                oldXError = xError;

                turret.setPower(xUt);

                telemetry.addData("Angle error", xError);
                telemetry.addData("PID out", xUt);
                telemetry.addData("PID integral", xIntegral);
                telemetry.addData("PID derivative", xDerivative);
        }

        telemetry.addData("Position", position);
        telemetry.addData("Loop time", loopTime);
    }
}
