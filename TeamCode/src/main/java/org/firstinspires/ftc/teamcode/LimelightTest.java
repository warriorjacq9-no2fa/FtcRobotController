package org.firstinspires.ftc.teamcode;

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
    private Limelight3A ll;
    private Servo servo;
    // 0-centered, radians
    private double position;
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastTime;
    private double xIntegral, xDerivative, oldXError;

    @Override
    public void init() {
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
    }
    double filter(double a, double b) {
        return 0.2 * a + (1 - 0.2) * b;
    }

    @Override
    public void loop() {
        LLResult res = ll.getLatestResult();
        double xError = oldXError;
        if(res != null && res.isValid()) {
            xError = AngleUnit.DEGREES.toRadians(res.getTx());
        }

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        xIntegral += xError * loopTime;

        xDerivative = filter((xError - oldXError) / loopTime, xDerivative);

        double xUt = Kp * xError + Ki * xIntegral + Kd * xDerivative;

        oldXError = xError;

        position += xUt;

        if(position > MAX_POSITION)
            position = MAX_POSITION;
        else if(position < MIN_POSITION)
            position = MIN_POSITION;

        double servoPosition =
                (position - MIN_POSITION) / (MAX_POSITION - MIN_POSITION);

        servo.setPosition(servoPosition);
        telemetry.addData("X error", xError);
        telemetry.addData("PID out", xUt);
        telemetry.addData("PID integral", xIntegral);
        telemetry.addData("PID derivative", xDerivative);
        telemetry.addData("Loop time", loopTime);
        telemetry.update();
    }
}
