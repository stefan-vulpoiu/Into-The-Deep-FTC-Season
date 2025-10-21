package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
class SampleSensor(hardwareMap: HardwareMap) {
    companion object {
        @JvmField
        var THRESHOLD: Double = 30.0

        @JvmField
        var MIN_DISTANCE: Double = 0.5

        @JvmField
        var MAX_DISTANCE: Double = 1.5

        @JvmField
        var RED_MATCH: LABColor = LABColor(48.0, 39.0, 44.0)

        @JvmField
        var BLUE_MATCH: LABColor = LABColor(39.0, 39.0, -76.0)

        @JvmField
        var YELLOW_MATCH: LABColor = LABColor(49.7, -22.6, 48.4)
    }

    var deltaRed: Double = 0.0
    var deltaBlue: Double = 0.0
    var deltaYellow: Double = 0.0

    private var sensor: RevColorSensorV3 =
        hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")

    val sampleType: SampleType
        get() {
            val colors = let {
                sensor.normalizedColors
                sensor.normalizedColors
                sensor.normalizedColors
            }

            val color = LABColor(colors)
            val distance = sensor.getDistance(DistanceUnit.CM)

            deltaRed = RED_MATCH.deltaE(color)
            deltaBlue = BLUE_MATCH.deltaE(color)
            deltaYellow = YELLOW_MATCH.deltaE(color)

            if (distance in MIN_DISTANCE..MAX_DISTANCE) {
                if ((deltaRed < deltaBlue && deltaRed < deltaYellow) && deltaRed <= THRESHOLD) {
                    return SampleType.Red
                } else if ((deltaBlue < deltaRed && deltaBlue < deltaYellow) && deltaBlue <= THRESHOLD) {
                    return SampleType.Blue
                } else if ((deltaYellow < deltaRed && deltaYellow < deltaBlue) && deltaYellow <= THRESHOLD) {
                    return SampleType.Yellow
                }
            }

            return SampleType.None
        }

    val color: LABColor
        get() {
            val colors = sensor.normalizedColors
            return LABColor(colors)
        }

    fun getDistance(unit: DistanceUnit?): Double {
        return sensor.getDistance(unit)
    }

    enum class SampleType {
        Blue,
        Red,
        Yellow,
        None;

        override fun toString(): String {
            return when (this) {
                Blue -> "Blue"
                Red -> "Red"
                Yellow -> "Yellow"
                else -> "None"
            }
        }
    }
}
