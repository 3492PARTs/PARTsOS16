#!/bin/sh -l

# Make sure that Gradlew is executable.
chmod +x gradlew

# Fix ant violations with lib
./gradlew :PARTsLib:spotlessApply

# Start simulation.
./gradlew simulateJavaRelease &
sim_pid=$!

# Wait for simulation to start.
sleep 10

# Kill the simulation.
kill $sim_pid