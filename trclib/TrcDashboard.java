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

/**
 * This interface provides a platform independent way to display info to the dashboard. It is mainly for TrcLib
 * which is platform agnostic. A platform dependent class will implement methods in this interface.
 */
public abstract class TrcDashboard
{
    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format  specifies the format string.
     * @param args    specifies variable number of substitution arguments.
     */
    public abstract void displayPrintf(int lineNum, String format, Object... args);

    protected static TrcDashboard instance = null;

    /**
     * This method allows any class to get an instance of the dashboard so that it can display information on its
     * display.
     *
     * @return global instance of the dashboard object.
     */
    public static TrcDashboard getInstance()
    {
        return instance;
    }   //getInstance

}   //class HalDashboard
