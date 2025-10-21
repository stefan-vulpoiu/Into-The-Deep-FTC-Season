package org.firstinspires.ftc.teamcode.tuners

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.EXTENDO_kD
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.EXTENDO_kP
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.LIFT_ATTACK
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem.extendoPosition
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config
@TeleOp(name = "Extendo Tuner", group = "Tuners")
class ExtendoTuner : TeleOpBase() {
    companion object {
        @JvmField
        var TARGET = 0.0
    }

    private lateinit var extendoMotor: MotorEx

    private lateinit var liftServo: Servo

    private val extendoController = PIDFController(
        kP = EXTENDO_kP,
        kD = EXTENDO_kD,
        setPointTolerance = 10.0
    )

    override fun initialize() {
        extendoMotor = MotorEx("extendo")
        extendoMotor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        extendoMotor.motor.setCurrentAlert(4.5, CurrentUnit.AMPS)
        extendoMotor.direction = DcMotorSimple.Direction.FORWARD
        extendoMotor.resetEncoder()

        liftServo = OpModeData.hardwareMap.get(Servo::class.java, "int_lift")
        liftServo.direction = Servo.Direction.REVERSE
    }

    override fun onStartButtonPressed() {
        liftServo.position = LIFT_ATTACK
    }

    override fun update() {
        extendoController.kP = EXTENDO_kP
        extendoController.kD = EXTENDO_kD

        if (TARGET != extendoController.target) {
            extendoController.target = TARGET
            extendoController.reset()
        }

        extendoMotor.power =
            extendoController.calculate(extendoMotor.currentPosition.extendoPosition)

        telemetry.addData("Position", extendoMotor.currentPosition.extendoPosition)
        telemetry.addData("Current", extendoMotor.motor.getCurrent(CurrentUnit.AMPS))
        telemetry.addData(
            "At position",
            extendoController.atTarget(extendoMotor.currentPosition.extendoPosition)
        )
    }
}