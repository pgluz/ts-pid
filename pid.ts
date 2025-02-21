// PIDController class to implement a PID control algorithm
class PIDController {
    private kp: number; // Proportional gain
    private ki: number; // Integral gain
    private kd: number; // Derivative gain

    private previousError: number; // Previous error value for derivative calculation
    private integral: number; // Accumulated integral value

    private setPoint: number; // Desired target value

    constructor(kp: number, ki: number, kd: number) {
        this.kp = kp; // Initialize proportional gain
        this.ki = ki; // Initialize integral gain
        this.kd = kd; // Initialize derivative gain

        this.previousError = 0; // Initialize previous error to zero
        this.integral = 0; // Initialize integral to zero
        this.setPoint = 0; // Initialize setpoint to zero
    }

    // Method to set the desired target value (setpoint)
    public setSetPoint(setPoint: number): void {
        this.setPoint = setPoint;
    }

    // Method to calculate the control output based on the current process variable (measured value)
    public calculate(currentValue: number, deltaTime: number): number {
        // Calculate the error as the difference between setpoint and current value
        const error = this.setPoint - currentValue;

        // Calculate the integral term (accumulating error over time)
        this.integral += error * deltaTime;

        // Calculate the derivative term (rate of change of error)
        const derivative = (error - this.previousError) / deltaTime;

        // Calculate the PID output using the formula:
        // output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        const output = (this.kp * error) + (this.ki * this.integral) + (this.kd * derivative);

        // Update previous error for next iteration
        this.previousError = error;

        return output; // Return the calculated control output
    }
}

// Example usage of the PIDController class
const pid = new PIDController(1.0, 0.1, 0.05); // Create a PID controller with specific gains

pid.setSetPoint(100); // Set desired target value to 100

// Simulate a process variable and time step for demonstration
let currentValue = 90;
const deltaTime = 1; // Time step in seconds

// Run the PID controller in a loop for demonstration purposes
for (let i = 0; i < 10; i++) {
    const output = pid.calculate(currentValue, deltaTime); // Calculate control output

    console.log(`Iteration ${i + 1}:`);
    console.log(`Current Value: ${currentValue}`);
    console.log(`Control Output: ${output}`);

    currentValue += output * deltaTime; // Simulate process variable response to control output
}
