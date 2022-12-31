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

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashMap;

/**
 * This class implements a priority indicator device that supports priority list. A priority list specifies a list of
 * indicator patterns in priority order. This means that if the indicator is set to a given pattern, it will be updated
 * only if the pattern being set has a higher priority than the pattern that is already active. This allows the
 * indicator to be used to display important status that will not be overwritten by unimportant status change. This
 * class is intended to be extended by a device dependent subclass that provides device dependent methods to set and
 * get indicator patterns.
 *
 * @param <T> specifies the device dependent indicator pattern type.
 */
public abstract class TrcPriorityIndicator<T>
{
    protected static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    protected static final boolean debugEnabled = false;

    /**
     * This method gets the current set pattern.
     *
     * @return currently set pattern.
     */
    public abstract T getPattern();

    /**
     * This method sets the pattern to the physical indicator device in a device dependent way.
     *
     * @param pattern specifies the indicator pattern. If null, turn off the indicator pattern.
     */
    public abstract void setPattern(T pattern);

    /**
     * This class implements the pattern state. It contains the pattern and the state if the pattern is active or not.
     */
    private class PatternState
    {
        final T pattern;
        boolean enabled;
        double expiredTime;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the pattern.
         * @param enabled specifies the initial state of the pattern.
         * @param expiredTime specifies expired time after which the pattern is turned OFF, zero if no expired time.
         */
        public PatternState(T pattern, boolean enabled, double expiredTime)
        {
            this.pattern = pattern;
            this.enabled = enabled;
            this.expiredTime = expiredTime;
        }   //PatternState

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the indicator pattern.
         */
        public PatternState(T pattern)
        {
            this(pattern, false, 0.0);
        }   //PatternState

        /**
         * This method returns the string representation of the LED pattern state.
         *
         * @return string representation of the LED pattern state.
         */
        @Override
        public String toString()
        {
            return pattern + "/" + enabled + ":" + expiredTime;
        }   //toString

    }   //class PatternState

    private final HashMap<String, T> namedPatternMap = new HashMap<>();
    private final String instanceName;
    private final TrcTaskMgr.TaskObject indicatorTaskObj;
    private PatternState[] patternPriorities = null;
    private boolean taskEnabled = false;
    private double nextTaskRunTime;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcPriorityIndicator(String instanceName)
    {
        this.instanceName = instanceName;
        indicatorTaskObj = TrcTaskMgr.createTask(instanceName, this::indicatorTask);
    }   //TrcPriorityIndicator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables the Indicator Task that periodically updates the indicator states.
     *
     * @param enabled specifies true to enable task, false to disable.
     */
    private void setTaskEnabled(boolean enabled)
    {
        if (enabled && !taskEnabled)
        {
            // Enabling task.
            indicatorTaskObj.registerTask(TrcTaskMgr.TaskType.OUTPUT_TASK);
            nextTaskRunTime = TrcUtil.getCurrentTime();
        }
        else if (!enabled && taskEnabled)
        {
            // Disabling task.
            indicatorTaskObj.unregisterTask();
        }

        taskEnabled = enabled;
    }   //setTaskEnabled

    /**
     * This method prints the Pattern Priority table to the given trace output for debugging purpose.
     *
     * @param tracer specifies the tracer object to use to print the info.
     */
    public synchronized void printPatternPriorityTable(TrcDbgTrace tracer)
    {
        final String funcName = "printPatternPriorityTable";
        StringBuilder msg = new StringBuilder("PatternPriorities=");

        if (patternPriorities != null)
        {
            for (PatternState state: patternPriorities)
            {
                msg.append(" ");
                msg.append(state);
            }
            tracer.traceInfo(funcName, msg.toString());
        }
    }   //printPatternPriorityTable

    /**
     * This method turns the indicator off.
     */
    public void reset()
    {
        setPattern(null);
    }   //reset

    /**
     * This method enables/disables the pattern in the priority list.
     *
     * @param pattern specifies the pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     * @param expiredTime specifies the time in seconds the LED stays ON, zero if stays on indefinitely.
     *        Only applicable when enabled is true.
     */
    public synchronized void setPatternState(T pattern, boolean enabled, double expiredTime)
    {
        final String funcName = "setPatternState";
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "pattern=%s,enabled=%s,index=%d", pattern, enabled, index);
        }

        if (index != -1)
        {
            patternPriorities[index].enabled = enabled;
            patternPriorities[index].expiredTime = 0.0;
            if (enabled && expiredTime > 0.0)
            {
                patternPriorities[index].expiredTime = TrcUtil.getCurrentTime() + expiredTime;
            }
            updateIndicator();
        }
    }   //setPatternState

    /**
     * This method enables/disables the pattern in the priority list.
     *
     * @param pattern specifies the pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     */
    public void setPatternState(T pattern, boolean enabled)
    {
        setPatternState(pattern, enabled, 0.0);
    }   //setPatternState

    /**
     * This method enables/disables the pattern in the priority list.
     *
     * @param patternName specifies the name of the pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     * @param expiredTime specifies the time in seconds the LED stays ON, zero if stays on indefinitely.
     *        Only applicable when enabled is true.
     * @throws IllegalAccessError when patternName is not found in the map.
     */
    public synchronized void setPatternState(String patternName, boolean enabled, double expiredTime)
    {
        setPatternState(namedPatternMap.get(patternName), enabled, expiredTime);
    }   //setPatternState

    /**
     * This method enables/disables the pattern in the priority list.
     *
     * @param patternName specifies the name of the pattern in the priority list.
     * @param enabled specifies true to turn the pattern ON, false to turn it OFF.
     * @throws IllegalAccessError when patternName is not found in the map.
     */
    public void setPatternState(String patternName, boolean enabled)
    {
        setPatternState(namedPatternMap.get(patternName), enabled, 0.0);
    }   //setPatternState

    /**
     * This method returns the pattern state if it is in the priority list. If the pattern is not in the list,
     * it returns false.
     *
     * @param pattern specifies the pattern in the priority list.
     * @return true if the pattern is ON, false if it is OFF.
     */
    public synchronized boolean getPatternState(T pattern)
    {
        final String funcName = "getPatternState";
        boolean state = false;
        int index = getPatternPriority(pattern);

        if (index != -1)
        {
            state = patternPriorities[index].enabled;
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "pattern=%s,index=%d,state=%s", pattern, index, state);
        }

        return state;
    }   //getPatternState

    /**
     * This method returns the pattern state if it is in the priority list. If the pattern is not in the list,
     * it returns false.
     *
     * @param patternName specifies the name of the pattern in the priority list.
     * @return true if the pattern is ON, false if it is OFF.
     * @throws IllegalAccessError when patternName is not found in the map.
     */
    public synchronized boolean getPatternState(String patternName)
    {
        return getPatternState(namedPatternMap.get(patternName));
    }   //getPatternState

    /**
     * This method resets all pattern states in the pattern priority list and set the indicator device to non-active
     * state.
     */
    public synchronized void resetAllPatternStates()
    {
        if (patternPriorities != null)
        {
            for (PatternState state : patternPriorities)
            {
                state.enabled = false;
                state.expiredTime = 0.0;
            }

            reset();
        }
    }   //resetAllPatternStates

    /**
     * This method searches the given pattern priorities array for the given pattern. If found, its index is
     * the priority and will be returned. If the pattern is not found in the array, -1 will be return which also
     * means the lowest priority.
     *
     * @param pattern specifies the indicator pattern to be searched in the pattern priorities array.
     * @return the pattern priority if found, -1 if not found.
     */
    public synchronized int getPatternPriority(T pattern)
    {
        final String funcName = "getPatternPriority";
        int priority = -1;

        if (patternPriorities != null)
        {
            for (int i = 0; i < patternPriorities.length; i++)
            {
                if (pattern == patternPriorities[i].pattern)
                {
                    priority = i;
                    break;
                }
            }
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "pattern=%s,priority=%d", pattern, priority);
        }

        return priority;
    }   //getPatternPriority

    /**
     * This method sets the pattern priority list for operations that need it. The priority list must be sorted in
     * decreasing priorities.
     *
     * @param priorities specifies the pattern priority list or null to disregard the previously set list.
     */
    @SuppressWarnings("unchecked")
    public synchronized void setPatternPriorities(T[] priorities)
    {
        final String funcName = "setPatternPriorities";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "priorityList=%s", priorities == null ? "null" : Arrays.toString(priorities));
        }

        if (priorities != null)
        {
            PatternState[] oldPriorities = patternPriorities;
            patternPriorities = (PatternState[]) Array.newInstance(PatternState.class, priorities.length);

            namedPatternMap.clear();
            for (int i = 0; i < patternPriorities.length; i++)
            {
                patternPriorities[i] = new PatternState(priorities[i]);
                namedPatternMap.put(patternPriorities[i].pattern.toString(), patternPriorities[i].pattern);
            }

            // If we had a previous priority list, make sure patterns persist
            if (oldPriorities != null)
            {
                for (PatternState patternState : oldPriorities)
                {
                    if (patternState.enabled)
                    {
                        // This will silently fail if this pattern is not in the priority list
                        setPatternState(patternState.pattern, true, patternState.expiredTime);
                    }
                }
            }
            updateIndicator();
            setTaskEnabled(true);
        }
        else
        {
            setTaskEnabled(false);
            patternPriorities = null;
            namedPatternMap.clear();
            reset();
        }
    }   //setPatternPriorities

    /**
     * This method is called to update the pattern according to the patternPriorities list. It will turn on the
     * highest priority pattern if enabled. If none of the patterns in the priority list is enabled, it will set
     * the indicator device to non-active state.
     */
    private void updateIndicator()
    {
        final String funcName = "updateIndicator";
        T pattern = null;

        double currTime = TrcUtil.getCurrentTime();
        for (int i = 0; i < patternPriorities.length; i++)
        {
            // Going from highest priority and down to low.
            if (patternPriorities[i].enabled)
            {
                if (patternPriorities[i].expiredTime > 0.0 && currTime >= patternPriorities[i].expiredTime)
                {
                    patternPriorities[i].enabled = false;
                    patternPriorities[i].expiredTime = 0.0;
                }
                else if (pattern == null)
                {
                    // Highest priority pattern that's enabled and not expired.
                    pattern = patternPriorities[i].pattern;
                }
            }
        }
        //
        // Only set the pattern if it is not already active.
        //
        if (pattern != getPattern())
        {
            setPattern(pattern);
        }

        if (debugEnabled)
        {
            globalTracer.traceInfo(funcName, "pattern=%s", pattern);
        }
    }   //updateIndicator

    /**
     * This task when enabled runs every 100 msec to update the indicator states.
     *
     * @param taskType specifies the task type (not used).
     * @param runMode specifies the robot run mode (not used).
     */
    private void indicatorTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        double currTime = TrcUtil.getCurrentTime();

        if (currTime >= nextTaskRunTime)
        {
            // Runs every 100 msec.
            nextTaskRunTime = currTime + 0.1;
            updateIndicator();
        }
    }   //indicatorTask

}   //class TrcPriorityIndicator
