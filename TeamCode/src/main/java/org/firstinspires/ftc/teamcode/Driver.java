package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public class Driver {

    private static final boolean DEBUG = false;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_DEG = 0.5;

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

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeading() {
            return heading;
        }
    }
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Telemetry telemetry;
    double oldFlEncoder;
    double oldFrEncoder;
    double oldBlEncoder;
    double oldBrEncoder;
    double rotated;

    public Driver(
            DcMotor frontLeft, DcMotor frontRight,
            DcMotor backLeft, DcMotor backRight,
            Telemetry telemetry
            ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * RUN_WITHOUT_ENCODER still allows encoder
         * readings, but allows us to set raw
         * power values instead of desired velocities
         */
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Driver initialized");
        telemetry.update();
    }

    /*
     * This method needs to be called in a loop
     * We don't use a loop inside the method since
     * the main OpMode loop cannot block (force
     * the controller to wait)
     */
    /**
     * Drive to a relative position
     *
     * @param dPose Relative target pose
     * @param speed Speed to move with, 0 to 1
     * @param reset Resets encoder and rotation
     *              counters
     * @return Remaining relative target pose to
     * pass back to this method, or null if done
     */
    public Pose driveTo(Pose dPose, double speed, boolean reset) {
        if(reset) {
            oldFlEncoder = 0;
            oldFrEncoder = 0;
            oldBlEncoder = 0;
            oldBrEncoder = 0;
            rotated = 0;
        }
        if(dPose == null) return null;
        if(
                Math.abs(dPose.getX()) < TOLERANCE_M &&
                Math.abs(dPose.getY()) < TOLERANCE_M &&
                Math.abs(dPose.getHeading()) < TOLERANCE_DEG
        ) return null;

        double flEncoder = frontLeft.getCurrentPosition();
        double frEncoder = frontRight.getCurrentPosition();
        double blEncoder = backLeft.getCurrentPosition();
        double brEncoder = backRight.getCurrentPosition();

        double flRotations = (flEncoder - oldFlEncoder) * ENCODER_CPR;
        double frRotations = (frEncoder - oldFrEncoder) * ENCODER_CPR;
        double blRotations = (blEncoder - oldBlEncoder) * ENCODER_CPR;
        double brRotations = (brEncoder - oldBrEncoder) * ENCODER_CPR;

        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;

        /*
         * To determine distance moved using the
         * encoders we can reverse the standard
         * mecanum movement equations,
         * for example for strafing (left and right):
         * [ dfl, dfr ] = [ y,-y ]
         * [ dbl, dbr ] = [-y, y ]
         * Plugging in the equation for wheel movement,
         * d=rΔθ
         * we get (after reversing):
         * [ y,-y ] = [ rΔθfl, rΔθfr ]
         * [-y, y ] = [ rΔθbl, rΔθbr ]
         * This simplifies to:
         * [ y, y ] = [ rΔθfl,-rΔθfr ]
         * [ y, y ] = [-rΔθbl, rΔθbr ]
         * Then calculate average of each wheel:
         * Δy = (rΔθfl + -rΔθfr + -rΔθbl + rΔθbr) / 4
         * And factor out r:
         * Δy = (r / 4)(Δθfl + -Δθfr + -Δθbl + Δθbr)
         *
         * To calculate forward and backward movement
         * (X axis) we add all wheel rotations since
         * moving forward spins all wheels forward.
         * Rotation is a little different since we need
         * to account for the chassis dimensions.
         * Given the equation:
         * [ dfl, dfr ] = [ ϕ,-ϕ ]
         * [ dbl, dbr ] = [ ϕ,-ϕ ]
         * We get (using the same principle as above):
         * Δϕ = (r / 4)(Δθfl + -Δθfr + Δθbl + -Δθbr)
         * Given L = distance from center to wheels
         * along the X axis and W = distance from
         * center to wheels along the Y axis, we
         * divide by L + W to get rotation in radians:
         * Δϕ = (r / 4(L + W))(Δθfl + -Δθfr + Δθbl + -Δθbr)
         * And then convert to degrees:
         * Δϕ = (360 / 2π)((r / 4(L + W))(Δθfl + -Δθfr + Δθbl + -Δθbr))
         */
        double dx = (WHEEL_RADIUS_M / 4) * (flRotations + frRotations + blRotations + brRotations);
        double dy = (WHEEL_RADIUS_M / 4) * (flRotations - frRotations - blRotations + brRotations);
        double drx = (360 / (2 * Math.PI)) *
                (WHEEL_RADIUS_M / (4 * (CHASSIS_LENGTH_M / 2 + CHASSIS_WIDTH_M / 2))) *
                (flRotations - frRotations - blRotations + brRotations);

        rotated += drx;

        /*
         * Now we have the distance moved, but we need to
         * account for rotation by converting the
         * x and y distances using this calculation:
         * [ x' ] = [ cosϕ,-sinϕ ] [ x ]
         * [ y' ] = [ sinϕ, cosϕ ] [ y ]
         * Which simplifies to:
         * x' = cosϕ * x + -sinϕ * y
         * y' = sinϕ * x + cosϕ * y
         */
        double g_dx = Math.cos(rotated) * dx - Math.sin(rotated) * dy;
        double g_dy = Math.sin(rotated) * dx + Math.cos(rotated) * dy;

        Pose pose = new Pose(
                dPose.getX() + g_dx,
                dPose.getY() + g_dy,
                dPose.getHeading() + drx
        );

        double x = speed * Math.signum(pose.getX());
        double y = speed * Math.signum(pose.getY());
        double rx = speed * Math.signum(pose.getHeading());

        frontLeft.setPower(x + y + rx);
        frontRight.setPower(x - y - rx);
        backLeft.setPower(x - y + rx);
        backRight.setPower(x + y - rx);

        /*
         * These telemetry calls would dump a
         * massive amount of data, so we only log
         * if we enable DEBUG when compiling
         */
        if(DEBUG) {
            telemetry.addData("Driver",
                    String.format(Locale.ENGLISH, "Moved %f,%f,%f", g_dx, g_dy, drx));
            telemetry.addData("Driver",
                    String.format(Locale.ENGLISH, "Wrote %f,%f,%f", x, y, rx));
            telemetry.update();
        }

        return pose;
    }
}
