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
    private static final String moduleName = "TrcPriorityIndicator";
    protected static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final boolean useGlobalTracer = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

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

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the pattern.
         * @param enabled specifies the initial state of the pattern.
         */
        public PatternState(T pattern, boolean enabled)
        {
            this.pattern = pattern;
            this.enabled = enabled;
        }   //PatternState

        /**
         * Constructor: Create an instance of the object.
         *
         * @param pattern specifies the indicator pattern.
         */
        public PatternState(T pattern)
        {
            this(pattern, false);
        }   //PatternState

    }   //class PatternState

    private final String instanceName;
    private TrcHashMap<String, T> namedPatternMap = null;
    private PatternState[] patternPriorities = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcPriorityIndicator(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer ?
                TrcDbgTrace.getGlobalTracer() :
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
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
     * This method sets the LED pattern map associating string names to LED patterns.
     *
     * @param namedPatternMap specifies the LED named pattern map to be set.
     */
    public void setNamedPatternMap(TrcHashMap<String, T> namedPatternMap)
    {
        this.namedPatternMap = namedPatternMap;
    }   //setNamedPatternMap

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
     */
    public void setPatternState(T pattern, boolean enabled)
    {
        final String funcName = "setPatternState";
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,enabled=%s,index=%d", pattern, enabled, index);
        }

        if (index != -1)
        {
            patternPriorities[index].enabled = enabled;
            updateIndicator();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
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
        if (namedPatternMap == null)
        {
            throw new IllegalStateException("Pattern name map has not been set.");
        }

        setPatternState(namedPatternMap.get(patternName), enabled);
    }   //setPatternState

    /**
     * This method returns the pattern state if it is in the priority list. If the pattern is not in the list,
     * it returns false.
     *
     * @param pattern specifies the pattern in the priority list.
     * @return true if the pattern is ON, false if it is OFF.
     */
    public boolean getPatternState(T pattern)
    {
        final String funcName = "getPatternState";
        boolean state = false;
        int index = getPatternPriority(pattern);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s,index=%d", pattern, index);
        }

        if (index != -1)
        {
            state = patternPriorities[index].enabled;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", state);
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
    public boolean getPatternState(String patternName)
    {
        if (namedPatternMap == null)
        {
            throw new IllegalStateException("Pattern name map has not been set.");
        }

        return getPatternState(namedPatternMap.get(patternName));
    }   //getPatternState

    /**
     * This method resets all pattern states in the pattern priority list and set the indicator device to non-active
     * state.
     */
    public void resetAllPatternStates()
    {
        final String funcName = "resetAllPatternStates";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (patternPriorities != null)
        {
            for (PatternState state : patternPriorities)
            {
                state.enabled = false;
            }

            reset();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
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
    public int getPatternPriority(T pattern)
    {
        final String funcName = "getPatternPriority";
        int priority = -1;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pattern=%s", pattern);
        }

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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", priority);
        }

        return priority;
    }   //getPatternPriority

    /**
     * This method sets the pattern priority list for operations that need it.
     *
     * @param priorities specifies the pattern priority list or null to disregard the previously set list.
     */
    @SuppressWarnings("unchecked")
    public void setPatternPriorities(T[] priorities)
    {
        final String funcName = "setPatternPriorities";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "priorityList=%s",
                priorities == null ? "null" : Arrays.toString(priorities));
        }

        if (priorities != null)
        {
            PatternState[] oldPriorities = patternPriorities;
            patternPriorities = (PatternState[]) Array.newInstance(PatternState.class, priorities.length);

            for (int i = 0; i < patternPriorities.length; i++)
            {
                patternPriorities[i] = new PatternState(priorities[i]);
            }

            // If we had a previous priority list, make sure patterns persist
            if (oldPriorities != null)
            {
                for (PatternState patternState : oldPriorities)
                {
                    if (patternState.enabled)
                    {
                        // This will silently fail if this pattern is not in the priority list
                        setPatternState(patternState.pattern, true);
                    }
                }
            }
            updateIndicator();
        }
        else
        {
            patternPriorities = null;
            reset();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
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

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        for (int i = patternPriorities.length - 1; i >= 0; i--)
        {
            if (patternPriorities[i].enabled)
            {
                pattern = patternPriorities[i].pattern;
                break;
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
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "! (pattern=%s)", pattern);
        }
    }   //updateIndicator

}   //class TrcPriorityIndicator
