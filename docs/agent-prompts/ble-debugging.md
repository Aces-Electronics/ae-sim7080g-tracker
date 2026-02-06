# Agent Prompt: Debug BLE Connection Issues

## Context

The AE-SIM7080G-S3 tracker has a BLE configuration interface that opens for 90 seconds on boot. Users have reported intermittent connection stability issues when trying to configure the device via the BLE app.

## Current Implementation

- **BLE Window**: 90 seconds after boot
- **Service UUID**: Custom service for tracker configuration
- **Characteristics**: Report interval, home location settings
- **Library**: NimBLE-Arduino 1.4.3
- **Platform**: ESP32-S3 with PSRAM

## Known Symptoms

1. BLE app sometimes fails to discover the device
2. Connection drops during configuration
3. Inconsistent behavior between Android and iOS devices
4. Some users report successful connections, others cannot connect at all

## Your Task

Investigate and resolve the BLE connection stability issues. Specifically:

1. **Analyze Current BLE Implementation**
   - Review the BLE service and characteristic definitions in `src/main.cpp`
   - Check for timing issues during the 90-second window
   - Verify advertising parameters and connection intervals

2. **Test Connection Stability**
   - Use BLE scanner apps to verify advertising visibility
   - Monitor connection events and disconnection reasons
   - Test with multiple devices (Android/iOS) if possible

3. **Identify Root Causes**
   - Check for conflicts with other peripherals (GPS, modem, accelerometer)
   - Verify PSRAM configuration doesn't interfere with BLE
   - Look for stack overflow or memory issues during BLE operations
   - Review NimBLE configuration and buffer sizes

4. **Implement Fixes**
   - Adjust BLE parameters for better stability
   - Add connection retry logic if needed
   - Improve error handling and user feedback
   - Consider extending the BLE window if timing is an issue

5. **Verify Resolution**
   - Test with the BLE app on multiple devices
   - Ensure configuration changes persist correctly
   - Document any changes to the BLE protocol

## Reference Files

- `/home/acea/ae-nv/ae-sim7080g-tracker/src/main.cpp` - Main firmware with BLE implementation
- `/home/acea/ae-nv/ae-sim7080g-tracker/platformio.ini` - Build configuration
- `/home/acea/ae-nv/ae-ble-app/` - Flutter BLE app (if needed for protocol verification)

## Success Criteria

- BLE device consistently discoverable during the 90-second window
- Stable connections from both Android and iOS devices
- Configuration changes successfully applied and persisted
- Clear error messages when connection fails

## Additional Notes

The tracker uses the same ESP32-S3 chip as other AE devices (smart shunt, hut sensors), so you can reference their BLE implementations for comparison. The key difference is the SIM7080G modem, which may be causing interference or resource contention.
