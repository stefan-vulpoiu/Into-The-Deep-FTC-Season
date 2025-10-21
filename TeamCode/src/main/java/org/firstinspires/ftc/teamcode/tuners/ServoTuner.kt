package org.firstinspires.ftc.teamcode.tuners

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config("Servo Tuner")
@TeleOp(name = "Servo Tuner", group = "Tuners")
class ServoTuner : TeleOpBase() {
    companion object {
        @JvmField
        var SERVO_NAME = "test"

        @JvmField
        var INVERT = false

        @JvmField
        var POSITION = 0.0
    }

    private lateinit var servo: Servo

    override fun initialize() {
        servo = hardwareMap.get(Servo::class.java, SERVO_NAME)
    }

    override fun update() {
        servo.position = POSITION
        servo.direction = if (INVERT) Servo.Direction.REVERSE else Servo.Direction.FORWARD
    }
}