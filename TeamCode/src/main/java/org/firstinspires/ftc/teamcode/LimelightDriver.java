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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class LimelightDriver {
    /* Config variables */
    public static boolean DEBUG = true;
    /* Min and max position in radians, positive = CCW */
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
    private double targetPosition = 0;
    private double oldEncoder = 0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime memTrackTimer = new ElapsedTime();
    private double lastTime = 0;
    private double xIntegral = 0, xDerivative = 0, oldXError = 0;

    public enum FiducialSets {
        RED_FAR,
        RED_CLOSE,
        BLUE_CLOSE,
        BLUE_FAR
    }

    private ArrayList<Integer>[] fiducialSets = new ArrayList[] {
            new ArrayList<>(Arrays.asList(30, 31, 32, 33)),
            new ArrayList<>(Arrays.asList(34, 35, 36, 37)),
            new ArrayList<>(Arrays.asList(38, 39, 40, 41)),
            new ArrayList<>(Arrays.asList(42, 43, 44, 45)),
    };
    private FiducialSets currentFiducialSet;

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

    Pose targetPose = new Pose(0, 0, 0);

    public void enterManualMode(double targetPosition) {
        this.targetPosition = targetPosition;
        state = LimelightState.LL_IDLE;
    }

    public void exitManualMode() {
        if(state == LimelightState.LL_IDLE)
            state = LimelightState.LL_MEM;
    }

    public void setFiducialSet(FiducialSets set) {
        currentFiducialSet = set;
    }

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
                // Just do nothing here
                break;

            case LL_MEM:
                if(res != null && res.isValid()) {
                    state = LimelightState.LL_TRACK; // We got a lock, start tracking
                    break;
                }

                Pose currentPose = driver.getCurrentPose();
                Pose currentVelocity = driver.getCurrentVelocity();

                double curVelTargetSpace = getPerpendicularVelocity(targetPose, currentVelocity);

                double targetAngle = Math.atan2(targetPose.y, targetPose.x);
                if(PROJECTILE_SPEED > 0) targetAngle += Math.asin(curVelTargetSpace / PROJECTILE_SPEED);
                targetAngle = normalize(targetAngle);
                double currentAngle = normalize(currentPose.heading + position);

                targetPosition = normalize(position - (currentAngle - targetAngle));

                if(DEBUG) {
                    telemetry.addData("Memory track target X", targetPose.x);
                    telemetry.addData("Memory track target Y", targetPose.y);
                    telemetry.addData("Memory track target angle", targetAngle);
                    telemetry.addData("Memory track current angle", currentAngle);
                }

                // Adjust stored target position for movement
                targetPose = new Pose(
                        targetPose.x - currentVelocity.x,
                        targetPose.y - currentVelocity.y,
                        0
                );
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

                for(LLResultTypes.FiducialResult fid: fidRes) {
                    if(fiducialSets[currentFiducialSet.ordinal()].contains(fid.getFiducialId())) {
                        found = true;
                        Position pos = fid.getTargetPoseCameraSpace().getPosition();
                        targetPose = new Pose(pos.x, pos.y, 0);
                        targetPosition = normalize(position - fid.getTargetXDegrees());
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
                break;
        }

        if(targetPosition > MAX_POSITION) {
            targetPosition = MAX_POSITION;
        }
        if(targetPosition < MIN_POSITION) {
            targetPosition = MIN_POSITION;
        }

        double xError = targetPosition - position;

        xIntegral += xError * loopTime;

        xDerivative = filter((xError - oldXError) / loopTime, xDerivative);

        double xUt = pid.p * xError + pid.i * xIntegral + pid.d * xDerivative;

        oldXError = xError;

        turret.setPower(xUt);

        if(DEBUG) {
            telemetry.addData("Angle error", xError);
            telemetry.addData("PID out", xUt);
            telemetry.addData("PID integral", xIntegral);
            telemetry.addData("PID derivative", xDerivative);
            telemetry.addData("Position", position);
            telemetry.addData("Loop time", loopTime);
        }
        telemetry.addData("Fiducial set", currentFiducialSet);
        telemetry.addData("State", state);
    }
}
