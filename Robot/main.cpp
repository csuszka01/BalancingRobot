#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

//#include <stdio.h>
#include <math.h>
#include <thread>
#include <unistd.h>
#include <limits.h>
#include <cstdlib>
#include <cctype>
//#include <sstream>
#include <string>
#include "ibalancingbot.cpp"
//#include "json.hpp"
//#include "pid.cpp"
#include "http_pid_set.cpp"
#include "influxdbwriter.cpp"
#define M_PI 3.14159265358979323846 /* pi */

#include "httplib.h"
#include <atomic>
#include "logger.h"

long double ref_time = 0.0;
long double update_ref_time = 0.0;
long long response_timeout = 1;
int FPS = 10;
long double dt = 1.0f / FPS;
long double delta_time;
long double update_delta_time;

long double CameraPosX = 0.0;
long double CameraPosY = 3;
long double CameraPosZ = 10.0;
long double ViewUpX = 0.0;
long double ViewUpY = 1.0;
long double ViewUpZ = 0.0;
long double CenterX = 0.0;
long double CenterY = 0.0;
long double CenterZ = 0.0;
bool follow_robot = false; // so that the camera will follow the robot or not

long double posx = 0;
long double posz = 0;

long double Theta = 0.0;
long double dtheta = 2 * M_PI / 100.0;
long double Radius = sqrt(pow(CameraPosX, 2) + pow(CameraPosZ, 2));

// GLUquadricObj* quadric = gluNewQuadric();
// gluQuadricNormals(quadric, GLU_SMOOTH);

long double rotation = 90.0;
long double posX = 0, posY = 0, posZ = 0;
long double move_unit = 3;
long double rate = 1.0f;
long double angle = -0.0f;
long double RotateX = 0.f, RotateY = 45.f;
IBalancingBot myBot;

long double speed = 0.0;
long double current_speed = 0.0;
long double turn = 0.0;
long double current_turn = 0.0;
bool use_pid = true;
long double F[] = {0.0, 0.0};

auto start_t = std::chrono::high_resolution_clock::now();

PID myPIDphi = PID();
PID myPIDx = PID();
PID myPIDpsi = PID();

HTTP_PID_SET myPIDset = HTTP_PID_SET("http://10.44.0.7:5000/pid", myPIDx, myPIDphi, myPIDpsi);

InfluxDBWriter influxdbwriter;
bool timeout_happened = false;
Logger::LogLevel active_log_level = Logger::LogLevel::NONE;
//std::mutex debug_log_mutex;

bool toppled = false;
bool reset_if_toppled = false;
std::atomic_bool trigger_reset = false;
int reset_countdown_seconds = 3;
long double time_toppled = 0;

using namespace std::literals::chrono_literals;

long double getElapsedTime() {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<long double, std::ratio<1>> duration = end - start_t;
    return duration.count();
}

std::string lowerString(std::string value) {
    for (char& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

//HTTP Server for handling on-demand reset
void run_http_server() {
    httplib::Server svr;

    // Listen for POST requests to /reset
    svr.Post("/reset", [](const httplib::Request&, httplib::Response& res) {
        trigger_reset = true;
        res.set_content("Reset initiated\n", "text/plain");
        Logger::info("robot reset initiated", {
            {"method", "http_reset"}
        });
    });

    svr.listen("0.0.0.0", 8080);
}

std::array<long double, 3> timedPidUpdate(HTTP_PID_SET& pids, long double x_val, long double phi_val, long double psi_val) {
    
    Logger::debug("PID request start", {
        {"axis", axis},
        {"current_value", static_cast<double>(current_value)},
        {"updated_delta_time", static_cast<double>(update_delta_time)},
        {"expected_dt",  static_cast<double>(dt)}
    });

    auto request_start = std::chrono::high_resolution_clock::now();
    try {
        auto result = pids.update(x_val, phi_val, psi_val, update_delta_time, dt, bot_phi);
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        Logger::debug("PID request ok", {
            {"axis", axis},
            {"duration_ms", static_cast<double>(duration.count())},
            {"result", static_cast<double>(result)}
        });
        return result;
    } catch (const std::exception& error) {
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        Logger::error("PID request exception", {
            {"axis", axis},
            {"duration_ms", static_cast<double>(duration.count())},
            {"error", error.what()},
            {"action", "throw"}
        });
        throw;
    } catch (...) {
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        Logger::error("PID request unknown exception", {
            {"axis", axis},
            {"duration_ms", static_cast<double>(duration.count())},
            {"action", "throw"}
        });
        throw;
    }
}

void initPIDs()
{
    // Function that initializes the PIDs parameters (Kp, Ki, Kd)

    myPIDphi.setKp(7.0);
    myPIDphi.setKi(0.1);
    myPIDphi.setKd(6.0);
    myPIDphi.setPoint(0);

    myPIDx.setKp(0.01);
    myPIDx.setKi(0.05 * dt);
    myPIDx.setKd(0.1 * dt);
    myPIDx.setPoint(0);

    myPIDpsi.setKp(1);
    myPIDpsi.setKi(1);
    myPIDpsi.setKd(0);
    myPIDpsi.setPoint(0);
}

void correction()
{
    Logger::debug("correcction start", {
        {"timeout_ms", response_timeout}
    });

    auto m = std::make_shared<std::mutex>();
    auto cv = std::make_shared<std::condition_variable>();
    auto finished = std::make_shared<bool>(false);

    // std::thread t([&cv, &copyMyPIDx, &copyMyPIDpsi, &copyMyPIDphi, &copyRotation, &copyF]() {
    std::thread t([cv, m, finished]()
                  {
        try {
            auto result = timedPidUpdate(myPIDset, myBot.xp, myBot.phi, -myBot.psip);
            //long double pidx_value = timedPidUpdate("x", myPIDx, myBot.xp);  // Pid over linear a speed
            //long double pidpsi_value = timedPidUpdate("psi", myPIDpsi, -myBot.psip);  // Pid over psi angular speed rotation
            //long double tilt = - pidx_value + myBot.phi;
            rotation = result[2];
            //long double pidphi_value = timedPidUpdate("phi", myPIDphi, tilt);  // pid over the pendulum angle phi
            //F[0] = -pidphi_value-rotation;
            //F[1] = -pidphi_value+rotation;
            F[0] = -result[0]-rotation;
            F[1] = -result[0]+rotation;
            
            {
                std::lock_guard<std::mutex> lock(*m);
                *finished = true;
            }
            cv->notify_one();
        }
        //Cetches JSON parse error, and sleeps until the timeout passes
        catch (const std::exception& error) {
            Logger::error("correction worker exception", {
                {"error", error.what()},
                {"action", "sleep"},
                {"sleep_ms", response_timeout}
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(response_timeout));
        }
        catch (...) {
            Logger::error("correction worker unknown exception", {
                {"action", "sleep"},
                {"sleep_ms", response_timeout}
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(response_timeout));
        } });

    t.detach();

    std::unique_lock<std::mutex> l(*m);
    
    if (!cv->wait_for(l, std::chrono::milliseconds(response_timeout), [&finished]() { return *finished; }))
    {
        Logger::warn("correction wait result", {
            {"main_thread_timeout", "true"},
            {"timeout_ms", response_timeout}
        });
        // t.join();
        // printf("runtime_error timeout\n");
        // throw std::runtime_error("Timeout");
        // t.join();
        std::cout << response_timeout << std::endl;
        throw std::exception();
        // throw std::runtime_error("Timeout");
    }
    Logger::debug("correction wait result", {
    {"main_thread_timeout", "false"}
    });
}

void timeoutCorrection()
{
    if (current_speed != speed)
    {
        current_speed = speed;
        myPIDx.setPoint(speed); // we only want to reset the PID when the speed changes
    }

    if (current_turn != turn)
    {
        current_turn = turn;
        myPIDpsi.setPoint(turn); // we only want to reset the PID when the rotation changes
    }

    //HTTP_PID copyMyPIDx(myPIDx);
    //HTTP_PID copyMyPIDpsi(myPIDpsi);
    //HTTP_PID copyMyPIDphi(myPIDphi);
    HTTP_PID_SET copyMyPIDset(myPIDset);
    long double copyRotation = rotation;
    long double copyF[2];
    copyF[0] = F[0];
    copyF[1] = F[1];
    try
    {
        correction();
    }
    // catch(std::runtime_error& e) {
    catch (...)
    {
        Logger::warn("timeoutCorrection exception", {
            {"action", "restoring PID state and motor forces"}
        });
        // printf("runtime_error timeout\n");
        std::this_thread::sleep_for(10ms);
        timeout_happened = true;
        //myPIDx = copyMyPIDx;
        //myPIDpsi = copyMyPIDpsi;
        //myPIDphi = copyMyPIDphi;
        myPIDset = copyMyPIDset;
        rotation = copyRotation;
        F[0] = copyF[0];
        F[1] = copyF[1];
    }
}

void threadCorrection()
{
    while (true)
    {
        // if (glutGet(GLUT_ELAPSED_TIME)-ref_time > (1.0/FPS)*1000) {
            if (getElapsedTime()-ref_time > 1.0/FPS) {
                correction();
            }
    }
}
void reset_robot(){
    // Reset robot
    initPIDs();
    myBot.phi = 0;
    myBot.phip = 0;
    toppled = false;
    trigger_reset = false;
}

void animation(){
    double current_time = getElapsedTime();
    delta_time = current_time-ref_time;
    update_delta_time = current_time-update_ref_time;

    bool topple_event = false; // If robot fell over in current iteration

    if (delta_time > 1.0/FPS){
        if (myBot.phi < 0.00001 && myBot.phi > -0.00001 && myBot.phip < 0.00001 && myBot.phip > -0.00001){
            Logger::info("All axis values zero.", {
                {"action", "init robot"}
            });
            myBot.initRobot();
        }

        if (myBot.phi <= 0.785 && myBot.phi >= -0.785){
            long double dst = 0;
            myBot.dynamics(delta_time, F);
            dst = (myBot.xp * delta_time);
            posx += dst*cos(myBot.psi);
            posz += (-dst*sin(myBot.psi));
            timeoutCorrection();
        } else if (myBot.phi > 0.785){
            topple_event = true;
            myBot.phi = 2.03;
        } else {
            topple_event = true;
            myBot.phi = -2.03;
        }
        // If toppled, start reset countdown
        if (topple_event && !toppled){
            time_toppled = current_time;
            toppled = true;
            // Log reset countdown start
            
            Logger::warn("robot fell over", {
                {"phi", myBot.phi}
            });
            if (reset_if_toppled) {
                Logger::info("robot reset initiated", {
                    {"method", "auto_reset"},
                    {"reset_countdown_seconds", reset_countdown_seconds}
                });
            }
        }
        // Reset if robot is toppled and ( countdown complete or reset triggered via HTTP )
        if (toppled && (reset_if_toppled && (current_time - time_toppled >= reset_countdown_seconds) || trigger_reset)) {
            // Reset robot
            reset_robot();
            topple_event = false;
            // Log reset complete
            Logger::info("robot reset complete");
        }

        if (toppled) {
            Logger::debug("robot fell over", {
                {"phi", myBot.phi}
            });
        }

        ref_time = getElapsedTime();
        if (timeout_happened == false) update_ref_time = getElapsedTime();
        influxdbwriter.Write(myBot.phi, timeout_happened, update_delta_time);
        timeout_happened = false;
    }
}

//Parse cli args and env vars
void parse_args(int argc, char **argv){
    // HTTP response timeout ms
    if (argc > 2)
    {
        response_timeout = std::atoll(argv[1]);
        FPS = std::atoll(argv[2]);
    }
    // Log level
    const char* log_level_env = std::getenv("LOG_LEVEL");
    if (log_level_env != nullptr) {
        active_log_level = Logger::parse_level(log_level_env);
        Logger::set_level(active_log_level);
    }
    if (argc > 3) {
        active_log_level = Logger::parse_level(argv[3]);
        Logger::set_level(active_log_level);
    }
    // Auto reset
    const char* auto_reset_env = std::getenv("RESET_COUNTDOWN_SECONDS");
    if (auto_reset_env != nullptr) {
        reset_if_toppled = true;
        reset_countdown_seconds = std::atoll(auto_reset_env);
    }
    if (argc > 4){
        reset_if_toppled = true;
        reset_countdown_seconds = std::atoll(argv[4]);
    }

    Logger::info("robot config", {
        {"response_timeout_ms", response_timeout},
        {"FPS", FPS},
        {"dt", static_cast<double>(dt)},
        {"auto_reset_enabled", reset_if_toppled},
        {"reset_countdown_seconds", reset_countdown_seconds},
        {"log_level", Logger::to_string(active_log_level)}
    });
}

int main(int argc, char **argv)
{
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    // influxdbwriter = InfluxDBWriter(std::string("http://influxdb.default.svc.cluster.local:8086"), std::string("robot"));
    influxdbwriter = InfluxDBWriter(std::string("http://influxdb.default.svc.cluster.local:8086"), std::string("robot"), std::string(hostname));
    // influxdbwriter = InfluxDBWriter("http://influxdb.default.svc.cluster.local:8086", "robot", hostname);

    //parse cli and env
    parse_args(argc, argv);

    dt = 1.0f / FPS;
    
    // std::thread correctionThread(threadCorrection);
    srand((unsigned)time(0));
    initPIDs();

    // Start the HTTP server for on-demand reset
    std::thread server_thread(run_http_server);
    server_thread.detach();
    Logger::info("HTTP Server listening", {
    {"port", 8080},
    {"endpoint", {
        {"path", "/reset"},
        {"request_type", "POST"}
            }
        }
    });

    while (1){
        animation();
    }
}
