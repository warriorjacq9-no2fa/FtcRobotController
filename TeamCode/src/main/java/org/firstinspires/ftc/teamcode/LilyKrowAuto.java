package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*We don't really know what we're doing, but we're trying*/

@Autonomous(name="Lily/KrowAuto")
public class LilyKrowAuto extends OpMode {

    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;

    private enum AutoState {
        INIT,
        DRIVE,
        TURN,
        COMPLETE
    }

    private AutoState state;

    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        state = AutoState.INIT;

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
    //a is forward, b is strafe, c is turn
    public void drive(double a, double b, double c) {
        frontLeft.setPower(a+b+c);
        frontRight.setPower(a+b-c);
        backLeft.setPower(a-b+c);
        backRight.setPower(a-b-c);
    }

    public void loop() {
        switch (state) {
            case INIT:
                drive(0.0, 0.0, 0.0);
                telemetry.addLine("INIT STATE");
                telemetry.update();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                state = AutoState.DRIVE;
                break;

            case DRIVE:
                drive(0.2, 0.0, 0.0);
                telemetry.addLine("DRIVE STATE");
                telemetry.update();
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                state = AutoState.TURN;
                break;

            case TURN:
                drive(0.0, 0.0, 0.2);
                telemetry.addLine("TURN STATE");
                telemetry.update();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                state = AutoState.COMPLETE;
                break;


            case COMPLETE:
                drive(0.0, 0.0, 0.0);
                telemetry.addLine("COMPLETE STATE");
                telemetry.update();
                break;
        }
    }
}
