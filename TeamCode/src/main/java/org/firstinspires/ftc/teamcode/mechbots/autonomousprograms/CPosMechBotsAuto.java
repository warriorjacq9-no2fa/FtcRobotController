package org.firstinspires.ftc.teamcode.mechbots.autonomousprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//Talk to Lily if you want to know where "position C" is on the field. They made it up. Eventually they'll get to telling everyone.

@Autonomous(name="CPosMechBotsAuto")
public class CPosMechBotsAuto extends OpMode {

    //speed = ?
    //set up intake to maybe get balls in auto
    //imu ?
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;
    private final ElapsedTime timer = new ElapsedTime();

    //Should this be a state machine? I don't know. Is it going to be a state machine? Yeah, unless someone tells me no.
    private enum AutoState {
        INIT,
        STRAFE,
        COMPLETE
    }
    private AutoState state = AutoState.INIT;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        //!!!! Not yet certain what these will need to be set to. Figure it out and uncomment before testing!!!
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Drive function. 'a' is for forward, 'b' is for strafe, 'c' is for turn. Positive goes right, negative goes left (I think).
    public void drive(double a, double b, double c) {
        frontLeft.setPower(a+b+c);
        frontRight.setPower(a+b-c);
        backLeft.setPower(a-b+c);
        backRight.setPower(a-b-c);
    }

    //The length of time it is in each state needs to be figured out through testing to make it work as intended!!!
    public void loop() {
        switch (state) {
            case INIT:
                if (timer.seconds() < 1) {
                    drive(0.0, 0.0, 0.0);
                    telemetry.addLine("INIT");
                    telemetry.update();
                }
                else {
                    state = AutoState.STRAFE;
                }
                break;

            case STRAFE:
                //Strafes left into the parking space
                if (timer.seconds() < 1) {
                    drive(0.0, -0.2, 0.0);
                    telemetry.addLine("STRAFE");
                    telemetry.update();
                }
                else {
                    state = AutoState.COMPLETE;
                    timer.reset();
                }
                break;

            case COMPLETE:
                drive(0.0, 0.0, 0.0);
                telemetry.addLine("PARKED");
                telemetry.update();
                break;
        }
    }
}
