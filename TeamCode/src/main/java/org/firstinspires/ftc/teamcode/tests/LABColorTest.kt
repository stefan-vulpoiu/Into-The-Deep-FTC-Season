package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.SampleSensor
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@TeleOp(name = "LAB Color Sensor Test", group = "Tests")
class LABColorTest : TeleOpBase() {
    private lateinit var sensor: SampleSensor

    override fun initialize() {
        sensor = SampleSensor(hardwareMap)
    }

    override fun update() {
        val color = sensor.color

        telemetry.addData("LAB", "%.3f, %.3f, %.3f", color.L, color.A, color.B)
        telemetry.addData("Distance", "%.3f", sensor.getDistance(DistanceUnit.CM))
        telemetry.addLine()

        telemetry.addData("Red Delta", sensor.deltaRed)
        telemetry.addData("Blue Delta", sensor.deltaBlue)
        telemetry.addData("Yellow Delta", sensor.deltaYellow)
        telemetry.addLine()

        telemetry.addData("Sample Detected", sensor.sampleType)
        telemetry.addLine()
    }
}