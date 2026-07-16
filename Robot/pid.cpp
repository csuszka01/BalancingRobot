#ifndef PID_CPP
#define PID_CPP

#include "json.hpp"

struct PID {
    long double Kp;
    long double Ki;
    long double Kd;
    long double Derivator;
    long double Integrator;
    long double Integrator_max;
    long double Integrator_min;

    long double set_point=0.0;
    long double error=0.0;
    
    long double P_value = 0.0;
    long double D_value = 0.0;
    long double I_value = 0.0;
    
    PID(long double P=7.0, long double I=0.1, long double D=6.0, long double Derivator=0, long double Integrator=0, long double Integrator_max=3, long double Integrator_min=-3): \
	Kp(P), Ki(I), Kd(D), Derivator(Derivator), Integrator(Integrator), Integrator_max(Integrator_max), Integrator_min(Integrator_min) {
        this->set_point=0.0;
        this->error=0.0;
    }

    PID(const PID& p) {
	Kp = p.Kp;
	Ki = p.Ki;
	Kd = p.Kd;
	Derivator = p.Derivator;
	Integrator = p.Integrator;
	Integrator_max = p.Integrator_max;
	Integrator_min = p.Integrator_min;
	
	set_point=p.set_point;
	error=p.error;
	
	P_value = p.P_value;
	D_value = p.D_value;
	I_value = p.I_value;
    }

    PID& operator=(const PID& p) {
	if(this != &p) {
	    Kp = p.Kp;
	    Ki = p.Ki;
	    Kd = p.Kd;
	    Derivator = p.Derivator;
	    Integrator = p.Integrator;
	    Integrator_max = p.Integrator_max;
	    Integrator_min = p.Integrator_min;
	    
	    set_point=p.set_point;
	    error=p.error;
	    
	    P_value = p.P_value;
	    D_value = p.D_value;
	    I_value = p.I_value;
	}
	return *this;
    }

    nlohmann::json to_json(long double current_value) const {
        return nlohmann::json::object({
            {"current_value", current_value},
            {"set_point", set_point},
            {"Kp", Kp}, {"Ki", Ki}, {"Kd", Kd},
            {"Integrator_min", Integrator_min},
            {"Integrator_max", Integrator_max},
            {"Integrator", Integrator},
            {"Derivator", Derivator},
            {"D_value", D_value}
        });
    }

    virtual long double update(long double current_value, long double dt, long double expected_dt) {
        // Calculate PID output value for given reference input and feedback
        this->error = this->set_point - current_value;
    
        // Proportional term
        this->P_value = this->Kp * this->error;
    
        // Filtered Derivative term
        long double tau = 3 * expected_dt;  // You can tune this constant
        long double alpha = dt / (dt + tau);
    
        // Calculate raw derivative and apply low-pass filter
        long double previous_error = this->Derivator;
        long double raw_derivative = (this->error - previous_error) / dt;
        this->D_value = (1.0 - alpha) * this->D_value + alpha * raw_derivative;
        long double filtered_derivative = this->D_value;  // for clarity
    
        this->Derivator = this->error;  // Save error for next loop
        long double D_term = this->Kd * filtered_derivative;
    
        // Integral term with manual clamping
        // this->Integrator += this->error * dt;
        this->Integrator += 0.5 * (this->error + previous_error) * dt;
        if (this->Integrator > this->Integrator_max) {
            this->Integrator = this->Integrator_max;
        } else if (this->Integrator < this->Integrator_min) {
            this->Integrator = this->Integrator_min;
        }
        this->I_value = this->Ki * this->Integrator;
    
        return this->P_value + this->I_value + D_term;
        
    }

    void setPoint(long double set_point) {
        // Initilize the setpoint of PID
        this->set_point = set_point;
        this->Integrator = 0;
        this->Derivator = 0;
    }

    void setIntegrator(long double Integrator) {
        this->Integrator = Integrator;
    }

    void setDerivator(long double Derivator) {
        this->Derivator = Derivator;
    }

    void setKp(long double P) {
        this->Kp = P;
    }

    void setKi(long double I) {
        this->Ki = I;
    }

    void setKd(long double D) {
        this->Kd = D;
    }

    long double getPoint() {
        return this->set_point;
    }
    
    long double getError() {
        return this->error;
    }

    long double getIntegrator() {
        return this->Integrator;
    }
    long double getDerivator() {
        return this->Derivator;
    }
};

#endif
