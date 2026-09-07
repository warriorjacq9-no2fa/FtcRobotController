package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Driver {

    public static final boolean DEBUG = true;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_RAD = 0.01;

    /* Counts per revolution, found on the product page for the motor */
    private static final double ENCODER_CPR = 384.5;
    private static final double WHEEL_RADIUS_M = 0.052;
    private static final double TRACK_WIDTH_M = 0.75; /* Front-back from wheel centers */
    private static final double WHEEL_BASE_M = 0.75; /* left-right from wheel centers */

    private static final double MAX_SPEED_RAD = (1000) * ((2 * Math.PI) / 60);

    /* PID constants */
    private static final double Kp = 0.5;
    private static final double Ki = 0.001;
    private static final double Kd = 0.005;


    /*
     * Since we are going to be using Limelight
     * cameras, we will also be using their
     * units in the autonomous code
     * Limelight uses meters for position and
     * degrees for rotation
     */
    public static class Pose {
        public double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;
    private final Telemetry telemetry;
    double oldFlEncoder;
    double oldFrEncoder;
    double oldBlEncoder;
    double oldBrEncoder;
    double heading;
    double oldHeading;
    ElapsedTime loopTimer = new ElapsedTime();
    double lastTime = 0;
    double xIntegral = 0;
    double yIntegral = 0;
    double rxIntegral = 0;
    double oldXError = 0;
    double oldYError = 0;
    double oldRxError = 0;

    public Driver(
            DcMotorEx frontLeft, DcMotorEx frontRight,
            DcMotorEx backLeft, DcMotorEx backRight,
            IMU imu,
            Telemetry telemetry
            ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        this.telemetry = telemetry;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * RUN_USING_ENCODER uses the motor
         * controller's integrated hardware
         * PID control, which helps to control
         * wheel slip
         */
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Driver initialized");
        telemetry.update();
    }

    public void loop() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("Heading", heading);
    }

    double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Drive to a given relative pose in the
     * global frame
     * @param dPose Relative target pose
     * @param speed Speed, in distanceUnits/sec
     * @param angleUnits Angle units for pose
     * @param distanceUnits Distance units for
     *                      pose and speed
     * @return Value to pass back to this
     * function next iteration, or null if done.
     * Essentially dPose - poseMoved
     */
    public Pose drive(
            Pose dPose, double speed,
            AngleUnit angleUnits, DistanceUnit distanceUnits
    ) {
        if(dPose == null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return null;
        }
        double x = distanceUnits.toMeters(dPose.x);
        double y = distanceUnits.toMeters(dPose.y);
        double rx = angleUnits.toRadians(dPose.heading);
        if(
                Math.abs(x) < TOLERANCE_M &&
                Math.abs(y) < TOLERANCE_M &&
                Math.abs(rx) < TOLERANCE_RAD
        ) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return null;
        }

        double flEncoder = frontLeft.getCurrentPosition();
        double frEncoder = frontRight.getCurrentPosition();
        double blEncoder = backLeft.getCurrentPosition();
        double brEncoder = backRight.getCurrentPosition();

        double flRadians = ((flEncoder - oldFlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double frRadians = ((frEncoder - oldFrEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double blRadians = ((blEncoder - oldBlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double brRadians = ((brEncoder - oldBrEncoder) / ENCODER_CPR) * 2 * Math.PI;

        double dx = (WHEEL_RADIUS_M / 4) * (flRadians + frRadians + blRadians + brRadians);
        double dy = (WHEEL_RADIUS_M / 4) * (flRadians - frRadians - blRadians + brRadians);

        double avgHeading = oldHeading + normalize(heading - oldHeading) / 2;
        double g_dx = Math.cos(avgHeading) * dx - Math.sin(avgHeading) * dy;
        double g_dy = Math.sin(avgHeading) * dx + Math.cos(avgHeading) * dy;

        Pose pose = new Pose(
                dPose.x - distanceUnits.fromMeters(g_dx),
                dPose.y - distanceUnits.fromMeters(g_dy),
                dPose.heading - angleUnits.fromRadians(normalize(heading - oldHeading))
        );

        double wheelSpeed = distanceUnits.toMeters(speed) / WHEEL_RADIUS_M;
        double xError = Math.cos(heading) * distanceUnits.toMeters(pose.x) +
                Math.sin(heading) * distanceUnits.toMeters(pose.y);
        double yError = -Math.sin(heading) * distanceUnits.toMeters(pose.x) +
                Math.cos(heading) * distanceUnits.toMeters(pose.y);
        double rxError = normalize(angleUnits.toRadians(pose.heading)) *
                        (TRACK_WIDTH_M / 2 + WHEEL_BASE_M / 2) / WHEEL_RADIUS_M;

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        xIntegral += xError * loopTime;
        yIntegral += yError * loopTime;
        rxIntegral += rxError * loopTime;

        double xUt = Kp * xError + Ki * xIntegral + Kd * ((xError - oldXError) / loopTime);
        double yUt = Kp * yError + Ki * yIntegral + Kd * ((yError - oldYError) / loopTime);
        double rxUt = Kp * rxError + Ki * rxIntegral + Kd * ((rxError - oldRxError) / loopTime);

        oldXError = xError;
        oldYError = yError;
        oldRxError = rxError;

        double wheelX = wheelSpeed * xUt;
        double wheelY = wheelSpeed * yUt;
        double wheelRx = wheelSpeed * rxUt;

        double flSpeed = wheelX + wheelY + wheelRx;
        double frSpeed = wheelX - wheelY - wheelRx;
        double blSpeed = wheelX - wheelY + wheelRx;
        double brSpeed = wheelX + wheelY - wheelRx;

        double maxSpeed = Math.max(
                Math.max(Math.abs(flSpeed), Math.abs(frSpeed)),
                Math.max(Math.abs(blSpeed), Math.abs(brSpeed))
        );

        if(maxSpeed > MAX_SPEED_RAD) {
            double ratio = MAX_SPEED_RAD / maxSpeed;
            flSpeed *= ratio;
            frSpeed *= ratio;
            blSpeed *= ratio;
            brSpeed *= ratio;
        }

        frontLeft.setVelocity(flSpeed, AngleUnit.RADIANS);
        frontRight.setVelocity(frSpeed, AngleUnit.RADIANS);
        backLeft.setVelocity(blSpeed, AngleUnit.RADIANS);
        backRight.setVelocity(brSpeed, AngleUnit.RADIANS);

        if(DEBUG) {
            telemetry.addData("Moved wheel", "%f %f %f %f",
                    flRadians, frRadians, blRadians, brRadians
            );
            telemetry.addData("Moved local", "%f %f %f", dx, dy, (oldHeading - heading));
            telemetry.addData("Moved global", "%f %f", g_dx, g_dy);
            telemetry.addData("Next global", "%f %f %f",
                    distanceUnits.toMeters(pose.x), distanceUnits.toMeters(pose.y),
                    angleUnits.toRadians(pose.heading)
            );
            telemetry.addData("Error", "%f %f %f", xError, yError, rxError);
            telemetry.addData("Wheel speeds", "%f %f %f %f",
                    flSpeed, frSpeed, blSpeed, brSpeed
            );
            telemetry.addData("PID out", "%f %f %f", xUt, yUt, rxUt);
            telemetry.addData("PID integrals", "%f %f %f", xIntegral, yIntegral, rxIntegral);
        }

        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;
        oldHeading = heading;

        return pose;
    }
}
