/*
 * Title: SpeedOutput
 *
 * Objective:
 *    This example demonstrates how to configure and read-in the High-Level
 *    Feedback mode "Speed Output" of a ClearPath motor.
 *
 *    This HLFB mode is available in ClearPath-MC series servos, in every
 *    ClearPath-MC operational mode except Ramp Up/Down to Selected Velocity.
 *
 * Description:
 *    This example reads the state of an attached ClearPath motor's HLFB output
 *    when configured for "Speed Output". During operation, the state of HLFB
 *    and calculated measured speed are written to the USB serial port.
 *
 *    This example does not enable the motor or command motion. Use the
 *    "Override Inputs" feature in MSP to command motion and see changes in the
 *    HLFB measurement.
 *
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2.  The connected ClearPath motor must be configured through the MSP software
 *    for an operational mode compatible with Speed Output HLFB mode (see above)
 * 3. The connected ClearPath motor must have its HLFB mode set to "Speed Output"
 *    (select Advanced>>High Level Feedback [Mode]... then choose "Speed Output"
 *    from the dropdown and hit the OK button).
 *    Select a 482 Hz PWM Carrier Frequency in this menu.
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 *
 * 
 * Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */
#include "ClearCore.h"
// Specify which motor to move.
// The INPUT_A_FILTER must match the Input A filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_FILTER 20
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor0 ConnectorM0
#define motor1 ConnectorM1
// Select the baud rate to match the target serial device
#define baudRate 9600
// Specify which serial to use: ConnectorUsb, ConnectorCOM0, or ConnectorCOM1.
#define SerialPort ConnectorUsb
// Declares our user-defined helper function, which is used to command torque.
// The definition/implementation of this function is at the bottom of the sketch.
bool CommandTorque(int8_t cT0,int8_t cT1);
void setup() {
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                      Connector::CPM_MODE_A_DIRECT_B_PWM);
    // Put the motor connector into the correct HLFB mode to read the Speed
    // Output PWM signal and convert it to percent of Max Speed.
    motor0.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
    motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Set up serial communication at a baud rate of baudRate (9600 bps) then
    // wait up to 5 seconds for a port to open.
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    int counter = 1;
    float sumSpeed = 0;
    SerialPort.PortOpen();
    while (!SerialPort && Milliseconds() - startTime < timeout) {
        continue;
    }
    motor0.EnableRequest(true);
    SerialPort.SendLine("Motor 0 Enabled");
    motor1.EnableRequest(true);
    SerialPort.SendLine("Motor 1 Enabled");
}

void loop() {
    uint32_t timeout = 5000;
    uint32_t startTime = Milliseconds();
    CommandTorque(0,0);    // See below for the detailed function definition.
}
/*------------------------------------------------------------------------------
 * CommandTorque
 *
 *    Command the motor to move using a torque of commandedTorque
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    torque)
 *
 * Parameters:
 *    int commandedTorque  - The torque to command
 *
 * Returns: True/False depending on whether the torque was successfully
 *    commanded.
 */
bool CommandTorque(int8_t cT0,int8_t cT1) {
    if (abs(cT0) > 255) {
        SerialPort.SendLine("M0 Move rejected, invalid torque requested");
        return false;
    }
    if (abs(cT1) > 255) {
        SerialPort.SendLine("M1 Move rejected, invalid torque requested");
        return false;
    }
    // Check if an alert is currently preventing motion
    if (motor0.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("M0 Motor status: 'In Alert'. Move Canceled.");
        return false;
    }
    if (motor1.StatusReg().bit.AlertsPresent) {
        SerialPort.SendLine("M1 Motor status: 'In Alert'. Move Canceled.");
        return false;
    }
    /*SerialPort.Send("Commanding M0 torque: ");
    SerialPort.SendLine(cT0);
    SerialPort.Send("Commanding M1 torque: ");
    SerialPort.SendLine(cT1);*/
    // Find the scaling factor of our torque range mapped to the PWM duty cycle
    // range (255 is the max duty cycle).
    // double scaleFactor = 255 / maxTorque;
    // Scale the torque command to our duty cycle range.
    uint8_t dutyRequest0 = abs(cT0); //* scaleFactor;
    uint8_t dutyRequest1 = abs(cT1);
    // Set input A to match the direction of torque.
    if (cT0 < 0) {
        motor0.MotorInAState(true);
    }
    else {
        motor0.MotorInAState(false);
    }
    // Ensures this delay is at least 2ms longer than the Input A filter
    // setting in MSP
    Delay_ms(2 + INPUT_A_FILTER);
    // Command the move
    motor0.MotorInBDuty(dutyRequest0);
    // Allow some time for HLFB to transition.
    while (motor0.HlfbState() != MotorDriver::HLFB_HAS_MEASUREMENT) {
        continue;
    }
    Delay_ms(1);
    if (cT1 < 0) {
        motor1.MotorInAState(true);
    }
    else {
        motor1.MotorInAState(false);
    }
    // Ensures this delay is at least 2ms longer than the Input A filter
    // setting in MSP
    Delay_ms(2 + INPUT_A_FILTER);
    // Command the move
    motor1.MotorInBDuty(dutyRequest1);
    // Waits for HLFB to assert (signaling a successful new torque output)
    while (motor1.HlfbState() != MotorDriver::HLFB_HAS_MEASUREMENT) {
        continue;
    }
    return true;
}
//------------------------------------------------------------------------------