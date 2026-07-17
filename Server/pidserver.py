# import flast module
from flask import Flask
from flask import request
import json
#import time
import scheddl
import os

rt = bool(os.getenv("RT", False))
runtime = int(os.getenv("RUNTIME", 100))
deadline = int(os.getenv("DEADLINE", 100))
period = int(os.getenv("PERIOD", 100))
threaded = bool(os.getenv("THREADED", False))

# instance of flask application
app = Flask(__name__)

# home route that returns below text when root url is accessed
@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"

def calculate_single_pid(pid_data, current_value, dt, alpha):
    """
    Helper to calculate a single PID step.
    """
    error = pid_data["set_point"] - current_value
    # Proportional
    P_value = pid_data["Kp"] * error
    # Filtered Derivative
    previous_error = pid_data["Derivator"]
    raw_derivative = (error - previous_error) / dt
    D_value = (1.0 - alpha) * pid_data["D_value"] + alpha * raw_derivative
    D_term = pid_data["Kd"] * D_value
    # Integrator
    Integrator = pid_data["Integrator"] + error * dt
    
    # Clamp Integrator
    if Integrator > pid_data["Integrator_max"]:
        Integrator = pid_data["Integrator_max"]
    elif Integrator < pid_data["Integrator_min"]:
        Integrator = pid_data["Integrator_min"]
        
    I_value = Integrator * pid_data["Ki"]
    
    # Output
    PID = P_value + I_value + D_term
    
    return {
        "error": error,
        "P_value": P_value,
        "D_value": D_value,
        "Derivator": error,
        "Integrator": Integrator,
        "I_value": I_value,
        "PID": PID
    }

@app.route("/pid", methods=['GET', 'POST'])
def pid():
    """
    Calculate PID output value for given reference input and feedback
    """
    if request.method == 'POST':
        d = request.get_data()
        
        d = json.loads(d.decode())
        
        try:
            #Filtered Derivative term
            dt = d["dt"]
            if dt <= 0:
                return json.dumps({"error": "dt must be greater than 0"}), 400
            
            tau = 3 * d["expected_dt"]
            alpha = dt / (dt + tau)

            pids = d["pids"]
            
            # 1. Calculate X
            x_results = calculate_single_pid(pids["x"], pids["x"]["current_value"], dt, alpha)
            
            # 2. Calculate Psi
            psi_results = calculate_single_pid(pids["psi"], pids["psi"]["current_value"], dt, alpha)
            
            # 3. Calculate Phi (dependent on X's newly calculated PID output)
            true_current_value_phi = -x_results["PID"] + pids["phi"]["current_value"]
            phi_results = calculate_single_pid(pids["phi"], true_current_value_phi, dt, alpha)
            
            ret_dict = {
                "x": x_results,
                "phi": phi_results,
                "psi": psi_results
            }

            return json.dumps(ret_dict)
        finally:
            
            pass

if __name__ == '__main__':  
    #param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    #os.sched_setscheduler(0, os.SCHED_FIFO, param)
    if rt:
        dl_args = (
            runtime  * 1000 * 1000, # runtime in nanoseconds
            deadline * 1000 * 1000, # deadline in nanoseconds
            period   * 1000 * 1000  # time period in nanoseconds
        )
        scheddl.set_deadline(*dl_args)
        app.run("0.0.0.0", port=5000, threaded=False)
    else:
        if not threaded:
            app.run("0.0.0.0", port=5000, threaded=False)
        else:
            app.run("0.0.0.0", port=5000, threaded=False, processes=16)
    #app.run("0.0.0.0", port=5000, threaded=False, processes=16)
