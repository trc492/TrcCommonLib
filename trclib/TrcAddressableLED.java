/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

import java.util.Arrays;
import java.util.Locale;

import trclib.TrcDbgTrace;
import trclib.TrcPriorityIndicator;

/**
 * This class implements a platform independent Addressable LED device. It is intended to be extended by a platform
 * dependent Addressable LED device to provides platform dependent methods to set the color pattern of the LED strip.
 */
public abstract class TrcAddressableLED extends TrcPriorityIndicator<TrcAddressableLED.Pattern>
{
    /**
     * This class contains information about an LED pattern. An LED pattern contains a pattern type, an array of colors
     * and a time interval between color changes for running patterns.
     */
    public static class Pattern
    {
        public enum Type
        {
            Fixed, Running, FadeInFadeOut
        }   //enum Type

        public Type type;
        public TrcColor[] colorPattern;
        public double runningInterval;

        /**
         * This method is called by all constructors to do common initialization.
         *
         * @param type specifies the pattern type.
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        private void commonInit(Type type, TrcColor[] colorPattern, double runningInterval)
        {
            this.type = type;
            this.colorPattern = colorPattern;
            this.runningInterval = runningInterval;
        }   //commonInit

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public Pattern(TrcColor[] colorPattern, Type type, double runningInterval)
        {
            commonInit(type, colorPattern, runningInterval);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param colorPattern specifies the color pattern as an array of colors, one for each pixel in the LED strip.
         */
        public Pattern(TrcColor[] colorPattern)
        {
            commonInit(Type.Fixed, colorPattern, 0.0);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         * @param type specifies the pattern type.
         * @param runningInterval specifies the time interval in seconds between each pattern change.
         */
        public Pattern(TrcColor color, int numLEDs, Type type, double runningInterval)
        {
            TrcColor[] fixedColors = new TrcColor[numLEDs];

            Arrays.fill(fixedColors, color);
            commonInit(type, fixedColors, runningInterval);
        }   //Pattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param color specifies the solid color in the color pattern.
         * @param numLEDs specifies the number of LEDs in the color pattern.
         */
        public Pattern(TrcColor color, int numLEDs)
        {
            this(color, numLEDs, Type.Fixed, 0.0);
        }   //Pattern

        @Override
        public String toString()
        {
            return String.format(Locale.US, "type=%s,color=%s,interval=%.3f",
                    type, Arrays.toString(colorPattern), runningInterval);
        }   //toString

    }   //class Pattern

    public abstract void updateLED(TrcColor[] colorPattern);

    protected final int numLEDs;
    protected Pattern currPattern = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numLEDs specifies the number of LEDs in the strip.
     */
    public TrcAddressableLED(String instanceName, int numLEDs)
    {
        super(instanceName);
        this.numLEDs = numLEDs;
    }   //TrcAddressableLED

    /**
     * This method sets the RGB color for the whole LED strip.
     *
     * @param red   specifies the red value (0-255).
     * @param green specifies the green value (0-255).
     * @param blue  specifies the blue value (0-255).
     */
    public void setRGB(int red, int green, int blue)
    {
        final String funcName = "setRGB";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "r=%d,g=%d,b=%d", red, green, blue);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currPattern = new Pattern(new TrcColor(red, green, blue), numLEDs);
        updateLED(currPattern.colorPattern);
    }   //setRGB

    /**
     * This method sets the HSV color for the whole LED strip.
     *
     * @param hue   specifies the hue (0-180).
     * @param sat   specifies the saturation (0-255).
     * @param value specifies the value (0-255).
     */
    public void setHSV(double hue, double sat, double value)
    {
        final String funcName = "setHSV";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "h=%d,s=%d,v=%d", hue, sat, value);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        // TODO: Implement TrcColor.hsvToRgb
        // updateLED(currPattern.colorPattern);
    }   //setHSV

    /**
     * This method sets the color for the whole LED strip.
     *
     * @param color specifies the color.
     */
    public void setColor(TrcColor color)
    {
        final String funcName = "setColor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=%s", color);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        currPattern = new Pattern(color, numLEDs);
        updateLED(currPattern.colorPattern);
    }   //setColor

    //
    // Implements TrcPriorityIndicator abstract methods.
    //

    /**
     * This method gets the current set pattern.
     *
     * @return currently set pattern.
     */
    @Override
    public Pattern getPattern()
    {
        final String funcName = "getPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", currPattern);
        }

        return currPattern;
    }   //getPattern

    /**
     * This method sets the pattern to the physical indicator device in a device dependent way.
     *
     * @param pattern specifies the indicator pattern. If null, turn off the indicator pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        final String funcName = "setPattern";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

        this.currPattern = pattern;
        updateLED(currPattern != null? currPattern.colorPattern: null);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPattern

}   //class TrcAddressableLED
