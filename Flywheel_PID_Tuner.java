package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Flywheel PID Tuner")
public class Flywheel_PID_Tuner extends OpMode {
    public DcMotorEx flywheelMotor;
    public DcMotorEx flywheelMotor2;
    private Servo leftdoor;
    private Servo rightdoor;
    private CRServo kicker;
    private CRServo kicker2;
    private DcMotor intake;

    public static double highVelocity = 1500;
    public static double lowVelocity = 1250;
    public static double P = 140.0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 11.8;

    private double curTargetVelocity = highVelocity;
    private boolean motorsRunning = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double loopTime = 0.0;

    private TelemetryManager panelsTelemetry;

    private double maxError = 0;
    private double avgError = 0;
    private int sampleCount = 0;
    private double errorSum = 0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        leftdoor = hardwareMap.servo.get("leftdoor");
        rightdoor = hardwareMap.servo.get("rightdoor");
        intake = hardwareMap.dcMotor.get("intake");
        kicker = hardwareMap.crservo.get("kicker");
        kicker2 = hardwareMap.crservo.get("kicker2");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        updatePIDFCoefficients();

        telemetry.addLine("=== Flywheel PID Tuner ===");
        telemetry.addLine("Open Panels in FTC Dashboard");
        telemetry.addLine("Graphs will auto-populate!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("X: Start/Stop Motors");
        telemetry.addLine("Y: Toggle High/Low Velocity");
        telemetry.addLine("RT: Test Rapid Fire");
        telemetry.addLine();
        telemetry.addLine("Tune PIDF in Dashboard Config");
        telemetry.update();
    }

    @Override
    public void loop() {
        kicker.setPower(-0.6);
        kicker2.setPower(0.6);
        if (gamepad1.x) {
            motorsRunning = !motorsRunning;
            if (motorsRunning) {
                runtime.reset();
                resetMetrics();
            } else {
                flywheelMotor.setVelocity(0);
                flywheelMotor2.setVelocity(0);
            }
            sleep(200);
        }

        if (gamepad1.y) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
            resetMetrics();
            sleep(200);
        }

        updatePIDFCoefficients();

        if (gamepad1.right_trigger > 0.7) {
            leftdoor.setPosition(0.85);
            rightdoor.setPosition(0.15);
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.7) {
            leftdoor.setPosition(0.56);
            rightdoor.setPosition(0.44);
            intake.setPower(1);
        }
        else if (gamepad1.a){
            intake.setPower(0.75);
        }
        else if (gamepad1.b){
            if (gamepad1.b){
        }
        } else {
            intake.setPower(0);
            leftdoor.setPosition(0.56);
            rightdoor.setPosition(0.44);
        }

        if (motorsRunning) {
            flywheelMotor.setVelocity(curTargetVelocity);
            flywheelMotor2.setVelocity(curTargetVelocity);
        }

        displayTelemetry();
    }

    private void updatePIDFCoefficients() {
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    private void resetMetrics() {
        maxError = 0;
        avgError = 0;
        sampleCount = 0;
        errorSum = 0;
    }

    private void displayTelemetry() {
        double vel1 = flywheelMotor.getVelocity();
        double vel2 = flywheelMotor2.getVelocity();
        double avgVel = (vel1 + vel2) / 2.0;
        double error = Math.abs(curTargetVelocity - avgVel);
        double errorPercent = (error / curTargetVelocity) * 100.0;

        if (motorsRunning) {
            errorSum += error;
            sampleCount++;
            avgError = errorSum / sampleCount;
            maxError = Math.max(maxError, error);
        }

        panelsTelemetry.addData("Target Velocity", curTargetVelocity);
        panelsTelemetry.addData("Current Velocity", avgVel);
        panelsTelemetry.addData("Motor 1 Velocity", vel1);
        panelsTelemetry.addData("Motor 2 Velocity", vel2);
        panelsTelemetry.addData("Error", error);
        panelsTelemetry.addData("Error Percent", errorPercent);

        panelsTelemetry.addData("P", P);
        panelsTelemetry.addData("I", I);
        panelsTelemetry.addData("D", D);
        panelsTelemetry.addData("F", F);

        panelsTelemetry.addData("Max Error", maxError);
        panelsTelemetry.addData("Avg Error", avgError);
        panelsTelemetry.addData("Motors Running", motorsRunning);
        panelsTelemetry.addData("Runtime", runtime.seconds());
        panelsTelemetry.addData("Loop Hz", getHz());

        String stability = errorPercent < 1 ? "EXCELLENT" :
                errorPercent < 2 ? "GOOD" :
                        errorPercent < 5 ? "ACCEPTABLE" : "POOR";
        panelsTelemetry.addData("Stability", stability);

        panelsTelemetry.update();
    }

    public double getHz() {
        double loop = System.nanoTime();
        double hz = 1000000000 / (loop - loopTime);
        loopTime = loop;
        return hz;
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}