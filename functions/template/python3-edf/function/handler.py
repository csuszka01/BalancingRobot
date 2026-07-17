import json

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


def handle(req):
    d = json.loads(req)
    #error = d["set_point"] - d["current_value"]

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


