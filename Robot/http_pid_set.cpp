#ifndef HTTP_PID_CPP
#define HTTP_PID_CPP

#include "pid.cpp"
#include "webclient.cpp"
#include <time.h>
#include <string>
#include <stdexcept>
#include <utility>
#include "json.hpp"
#include <array>

using json = nlohmann::json;

struct HTTP_PID_SET {
    std::string serveraddr;
    static WebClient wc;

    PID PIDx;
    PID PIDphi;
    PID PIDpsi;
    
    HTTP_PID_SET(std::string pidserver, const PID& x, const PID& phi, const PID& psi)
     :  serveraddr(std::move(pidserver)),
        PIDx(x),
        PIDphi(phi),
        PIDpsi(psi)
    {}
    
    HTTP_PID_SET(const HTTP_PID_SET&) = default;
    HTTP_PID_SET& operator=(const HTTP_PID_SET&) = default;
    
    std::array<long double, 3> update(long double x_val, long double phi_val, long double psi_val, long double dt, long double expected_dt) {
        struct timespec ts;
        timespec_get(&ts, TIME_UTC);

        json data = json::object({
            {"http_request_mode", 0}, // single http request mode  for updating all axis at once
            {"time", 1000000000 * ts.tv_sec + ts.tv_nsec},
            {"dt", dt},
            {"expected_dt", expected_dt},

            {"pids", json::object({
                {"x",   PIDx.to_json(x_val)},
                {"phi", PIDphi.to_json(phi_val)},
                {"psi", PIDpsi.to_json(psi_val)}
            })}
        });

        wc.post(to_string(data));

        json return_data;
        std::string response;
        try {
            response = wc.getResponse();
            if (response.empty()) {
                throw std::runtime_error("Server response empty");
            }

            return_data = json::parse(response); }
        catch (json::parse_error& error) {
            std::string err = error.what() + response;
            throw std::runtime_error(err);
        }
        if (return_data.contains("error")) {
                throw std::runtime_error("Dt was 0, no data returned");
            }
        if (!return_data.contains("x") || !return_data.contains("phi") || !return_data.contains("psi")) {
            throw std::runtime_error("Server response lacks expected PID keys! Raw response: " + return_data.dump());
        }

        PIDx.error = return_data["x"]["error"];
        PIDx.P_value = return_data["x"]["P_value"];
        PIDx.I_value = return_data["x"]["I_value"];
        PIDx.D_value = return_data["x"]["D_value"];
        PIDx.Integrator = return_data["x"]["Integrator"];
        PIDx.Derivator = return_data["x"]["Derivator"];

        PIDphi.error = return_data["phi"]["error"];
        PIDphi.P_value = return_data["phi"]["P_value"];
        PIDphi.I_value = return_data["phi"]["I_value"];
        PIDphi.D_value = return_data["phi"]["D_value"];
        PIDphi.Integrator = return_data["phi"]["Integrator"];
        PIDphi.Derivator = return_data["phi"]["Derivator"];

        PIDpsi.error = return_data["psi"]["error"];
        PIDpsi.P_value = return_data["psi"]["P_value"];
        PIDpsi.I_value = return_data["psi"]["I_value"];
        PIDpsi.D_value = return_data["psi"]["D_value"];
        PIDpsi.Integrator = return_data["psi"]["Integrator"];
        PIDpsi.Derivator = return_data["psi"]["Derivator"];
        
        long double PIDx_value = return_data["x"]["PID"];
        long double PIDphi_value = return_data["phi"]["PID"];
        long double PIDpsi_value = return_data["psi"]["PID"];

        return {PIDx_value, PIDphi_value, PIDpsi_value};
    }

    long double update_axis(PID& pid, long double current_value, long double dt, long double expected_dt){

        struct timespec ts;
        timespec_get(&ts, CLOCK_TAI);
        json data = json::object({
                        {"http_request_mode", 1}, //update single axis
                        {"time", 1000000000*ts.tv_sec+ts.tv_nsec},
                        {"current_value", current_value},
                        {"expected_dt", expected_dt},
                        {"dt", dt},
                        {"set_point", pid.set_point},
                        {"Kp", pid.Kp},
                        {"Ki", pid.Ki},
                        {"Kd", pid.Kd},
                        {"Integrator_min", pid.Integrator_min},
                        {"Integrator_max", pid.Integrator_max},
                        {"Integrator", pid.Integrator},
                        {"Derivator", pid.Derivator},
                        {"D_value", pid.D_value}
                    });
        wc.post(to_string(data));

        //std::stringstream buffer;
        /* buffer << "{" << "\"current_value\": " << current_value << ", \"set_point\": " \
            << this->set_point << ", \"error\": " << this->error \
            << ", \"Kp\": " << this->Kp << ", \"Ki\": " << this->Ki <<  ", \"Kd\": " << this->Kd \
            << ", \"Integrator_min\": " << this->Integrator_min << ", \"Integrator_max\": " << this->Integrator_max \
            << ", \"Integrator\": " << this->Integrator << ", \"Derivator\": " << this->Derivator << "}"; */
            //wc.post(buffer.str());
        
        //std::stringstream ss(wc.getResponse());
        json return_data;
        std::string response;
        try {
            response = wc.getResponse();
            if (response.empty()) {
                throw std::runtime_error("Server response empty");
            }
            return_data = json::parse(response); }
        catch (json::parse_error& error) {
            std::string err = error.what() + response;
            throw std::runtime_error(err);
        }
        pid.error = return_data["error"];
        pid.P_value = return_data["P_value"];
        pid.I_value = return_data["I_value"];
        pid.D_value = return_data["D_value"];
        pid.Integrator = return_data["Integrator"];
        pid.Derivator = return_data["Derivator"];
        
        //std::cout<<to_string(return_data)<<std::endl;
        //std::cout<<ss.str();
        return return_data["PID"];
    }
};



//WebClient HTTP_PID::wc = WebClient("http://pidserver.default.svc.cluster.local:5000/pid");
//WebClient HTTP_PID_SET::wc = WebClient("http://pidserver.openfaas-fn.svc.cluster.local:8080/pid");
WebClient HTTP_PID_SET::wc = WebClient("http://gateway.openfaas:8080/function/pidserver");

#endif
