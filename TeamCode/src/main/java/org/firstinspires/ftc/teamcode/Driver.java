package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class Driver {

    public static final boolean DEBUG = true;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_RAD = 0.01;

    /* Counts per revolution, found on the product page for the motor */
    private static final double ENCODER_CPR = 384.5;
    private static final double WHEEL_RADIUS_M = 0.052;
    /* TODO: measure chassis */
    private static final double CHASSIS_LENGTH_M = 0.300;
    private static final double CHASSIS_WIDTH_M = 0.300;


    /*
     * Since we are going to be using Limelight
     * cameras, we will also be using their
     * units in the autonomous code
     * Limelight uses meters for position and
     * degrees for rotation
     */
    public static class Pose {
        private final double x, y, heading;
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

        double g_dx = Math.cos(heading) * dx - Math.sin(heading) * dy;
        double g_dy = Math.sin(heading) * dx + Math.cos(heading) * dy;

        Pose pose = new Pose(
                dPose.x - distanceUnits.fromMeters(g_dx),
                dPose.y - distanceUnits.fromMeters(g_dy),
                dPose.heading - angleUnits.fromRadians(oldHeading - heading)
        );

        double localX = Math.cos(heading) * distanceUnits.toMeters(pose.x) +
                Math.sin(heading) * distanceUnits.toMeters(pose.y);
        double localY = -Math.sin(heading) * distanceUnits.toMeters(pose.x) +
                Math.cos(heading) * distanceUnits.toMeters(pose.y);

        // TODO: PID control
        double wheel_x = speed * Math.signum(localX);
        double wheel_y = speed * Math.signum(localY);
        double wheel_rx = speed * Math.signum(pose.heading);

        frontLeft.setVelocity(wheel_x + wheel_y + wheel_rx, AngleUnit.RADIANS);
        frontRight.setVelocity(wheel_x - wheel_y - wheel_rx, AngleUnit.RADIANS);
        backLeft.setVelocity(wheel_x - wheel_y + wheel_rx, AngleUnit.RADIANS);
        backRight.setVelocity(wheel_x + wheel_y - wheel_rx, AngleUnit.RADIANS);

        if(DEBUG) {
            telemetry.addData("Wheel", "%f %f %f %f",
                    flRadians, frRadians, blRadians, brRadians
            );
            telemetry.addData("Local", "%f %f %f", dx, dy, (oldHeading - heading));
            telemetry.addData("Global", "%f %f", g_dx, g_dy);
            telemetry.addData("Next global", "%f %f %f",
                    distanceUnits.toMeters(pose.x), distanceUnits.toMeters(pose.y),
                    angleUnits.toRadians(pose.heading)
            );
            telemetry.addData("Next local", "%f %f", localX ,localY);
            telemetry.addData("Heading", "%f", heading);
            telemetry.addData("Wrote", "%f %f %f", wheel_x, wheel_y, wheel_rx);
        }

        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;
        oldHeading = heading;

        return pose;
    }
}
