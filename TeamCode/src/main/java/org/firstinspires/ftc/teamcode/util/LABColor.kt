package org.firstinspires.ftc.teamcode.util

import android.graphics.Color
import androidx.annotation.ColorInt
import androidx.core.graphics.ColorUtils
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.util.Range

class LABColor {
    @JvmField
    var L: Double

    @JvmField
    var A: Double

    @JvmField
    var B: Double

    constructor(color: NormalizedRGBA) {
        @ColorInt val colorInt = Color.rgb(
            convertToRange(color.red.toDouble()),
            convertToRange(color.green.toDouble()),
            convertToRange(color.blue.toDouble())
        )

        val lab = DoubleArray(3)

        ColorUtils.colorToLAB(colorInt, lab)

        L = lab[0]
        A = lab[1]
        B = lab[2]
    }

    constructor(L: Double, A: Double, B: Double) {
        this.L = L
        this.A = A
        this.B = B
    }

    private fun convertToRange(value: Double): Int {
        return Range.clip(Math.round(value * 100 * 255).toInt(), 0, 255)
    }

    fun deltaE(comparing: LABColor): Double {
        val lab1 = doubleArrayOf(L, A, B)
        val lab2 = doubleArrayOf(comparing.L, comparing.A, comparing.B)

        return ColorUtils.distanceEuclidean(lab1, lab2)
    }
}