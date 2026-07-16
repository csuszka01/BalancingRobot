#ifndef HTTP_PID_CPP
#define HTTP_PID_CPP

#include "pid.cpp"
#include "webclient.cpp"
#include <time.h>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <utility>
#include "json.hpp"
#include <array>

using json = nlohmann::json;
using std::to_string;

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
        try {
            return_data = json::parse(wc.getResponse());
        }
        catch (json::parse_error& error) {
            throw std::runtime_error("parse error");
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

        //long double rotation = PIDpsi_value;

        //return {-PIDphi_value-rotation, -PIDphi_value+rotation};
        return {PIDx_value, PIDphi_value, PIDpsi_value}
    }
};

//WebClient HTTP_PID::wc = WebClient("http://pidserver.default.svc.cluster.local:5000/pid");
WebClient HTTP_PID_SET::wc = WebClient("http://pidserver.openfaas-fn.svc.cluster.local:8080/pid");

#endif
