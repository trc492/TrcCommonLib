/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * This class implements a diagnostics test collection. The client adds a bunch of diagnostics tests into the
 * collection and this class provides the mechanism to run the tests and get the results.
 */
public class TrcDiagnostics<T> implements Iterable<TrcDiagnostics.Test<T>>
{
    /**
     * This class implements a generic diagnostics test with a specified name and a group the test is associated
     * with. This class is intended to be extended by a more specific diagnostics test that provides the actual
     * method to perform test. A method is provided to run the test and update the test status.
     *
     * @param <T> specifies the group enum type.
     */
    public static abstract class Test<T>
    {
        /**
         * This method is called to run the diagnostics test once. If the test passed, a null is return.
         * Otherwise, an error message is returned.
         *
         * @return null if test passed, otherwise an error message is returned.
         */
        protected abstract String runTest();

        private static final String defErrorMsg = "Test disabled by conditional.";

        private final String testName;
        private final T testGroup;
        private final TrcUtil.DataSupplier<Boolean> conditional;
        private final boolean defStatus;
        private boolean testPassed = true;
        private String testError = null;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param name specifies the name of the test.
         * @param group specifies the test group the test is associated with.
         * @param conditional specifies the conditional method that determines whether the test will be run,
         *                    null if none specified in which case, the test will always run.
         * @param defStatus specifies the default test status to be returned if the test is not run because
         *                  conditional was false.
         */
        public Test(String name, T group, TrcUtil.DataSupplier<Boolean> conditional, boolean defStatus)
        {
            this.testName = name;
            this.testGroup = group;
            this.conditional = conditional == null? () -> true: conditional;
            this.defStatus = defStatus;
        }   //Test

        /**
         * Constructor: Create an instance of the object.
         *
         * @param name specifies the name of the test.
         * @param group specifies the test group the test is associated with.
         */
        public Test(String name, T group)
        {
            this(name, group, () -> true, false);
        }   //Test

        /**
         * This method returns the test object state.
         *
         * @return test state string.
         */
        @Override
        public String toString()
        {
            String str = "Test:" + testGroup + "/" + testName;
            str += testPassed? " passed": " failed (" + testError + ")";
            return str; 
        }   //toString

        /**
         * This method returns the test name.
         *
         * @return test name.
         */
        public String getTestName()
        {
            return testName;
        }   //getTestName

        /**
         * This method returns the test group the test is associated with.
         *
         * @return test group.
         */
        public T getTestGroup()
        {
            return testGroup;
        }   //getTestGroup

        /**
         * This method returns the test status.
         *
         * @return true if test has passed, false otherwise.
         */
        public boolean hasPassed()
        {
            return testPassed;
        }   //hasPassed

        /**
         * This method returns the test error message. If the test passed, null is returned.
         *
         * @return null if test passed, the error message otherwise.
         */
        public String getTestError()
        {
            return testError;
        }   //getTestError

        /**
         * This method runs the test and update the test status.
         */
        public void runTestAndUpdateStatus()
        {
            //
            // Call runTest only if we don't have a conditional or if conditional returned true.
            // If runTest was not called, then use defStatus as the default test status.
            //
            this.testError = conditional.get()? runTest(): defStatus? null: defErrorMsg;
            this.testPassed = testError == null;
        }   //runTestAndUpdateStatus

    }   //class Test

    private final List<Test<T>> testCollection = new ArrayList<>();

    /**
     * This method adds a test into the collection.
     *
     * @param test specifies the test to be added to the collection.
     */
    public void addTest(Test<T> test)
    {
        testCollection.add(test);
    }   //addTest

    /**
     * This method runs all tests in the collection and updates their status.
     */
    public void runAllTests()
    {
        testCollection.forEach(Test::runTestAndUpdateStatus);
    }   //runAllTests

    /**
     * This method returns a map of each test group and whether all tests in that group have passed.
     *
     * @return a map of the test groups and their test status.
     */
    public Map<T, Boolean> getTestGroupResults()
    {
        return testCollection.stream().collect(
                Collectors.groupingBy(
                    Test::getTestGroup,
                    Collectors.reducing(
                        true, // Show green by default
                        Test::hasPassed, // Show green if not faulted
                        Boolean::logicalAnd))); // Show green only if all are OK
    }   //getTestGroupResults

    //
    // Implements the Iterator interface.
    //
    @Override
    public Iterator<Test<T>> iterator()
    {
        return testCollection.iterator();
    }   //iterator

}   //class TrcDiagnostics
