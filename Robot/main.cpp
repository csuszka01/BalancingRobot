#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <stdio.h>
#include <math.h>
#include <thread>
#include <unistd.h>
#include <limits.h>
#include <cstdlib>
#include <cctype>
#include <sstream>
#include <string>
#include "ibalancingbot.cpp"
#include "pid.cpp"
#include "http_pid.cpp"
#include "influxdbwriter.cpp"
#define M_PI 3.14159265358979323846 /* pi */

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

HTTP_PID myPIDphi = HTTP_PID("http://10.44.0.7:5000/pid");
HTTP_PID myPIDx = HTTP_PID("http://10.44.0.7:5000/pid");
HTTP_PID myPIDpsi = HTTP_PID("http://10.44.0.7:5000/pid");
InfluxDBWriter influxdbwriter;
bool timeout_happened = false;
bool debug_mode = false;
std::mutex debug_log_mutex;

bool toppled = false;
bool reset_if_toppled = false;
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

// TODO: needs enum return value for different loglevel implementation
bool parseDebugFlag(const char* value) {
    if (value == nullptr) {
        return false;
    }

    std::string normalized = lowerString(std::string(value));
    return normalized == "1" || normalized == "true" ||
           normalized == "yes" || normalized == "on";
}

void debugLog(const std::string& message) {
    if (!debug_mode) {
        return;
    }

    std::lock_guard<std::mutex> lock(debug_log_mutex);
    std::cerr << "[robot-debug t=" << static_cast<double>(getElapsedTime())
              << "s] " << message << std::endl;
}

long double timedPidUpdate(const std::string& axis,
                           HTTP_PID& pid,
                           long double current_value) {
    std::ostringstream start_message;
    start_message << "PID " << axis << " request start"
                  << " current_value=" << static_cast<double>(current_value)
                  << " update_delta_time=" << static_cast<double>(update_delta_time)
                  << " expected_dt=" << static_cast<double>(dt);
    debugLog(start_message.str());

    auto request_start = std::chrono::high_resolution_clock::now();
    try {
        long double result = pid.update(current_value, update_delta_time, dt);
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        std::ostringstream success_message;
        success_message << "PID " << axis << " request ok"
                        << " duration_ms=" << static_cast<double>(duration.count())
                        << " result=" << static_cast<double>(result);
        debugLog(success_message.str());
        return result;
    } catch (const std::exception& error) {
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        std::ostringstream error_message;
        error_message << "PID " << axis << " request exception"
                      << " duration_ms=" << static_cast<double>(duration.count())
                      << " error=\"" << error.what() << "\"";
        debugLog(error_message.str());
        throw;
    } catch (...) {
        auto request_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<long double, std::milli> duration =
            request_end - request_start;

        std::ostringstream error_message;
        error_message << "PID " << axis << " request unknown exception"
                      << " duration_ms=" << static_cast<double>(duration.count());
        debugLog(error_message.str());
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
    std::ostringstream start_message;
    start_message << "correction start timeout_ms=" << response_timeout;
    debugLog(start_message.str());

    std::mutex m;
    std::condition_variable cv;

    // correctionReturnStruct ret;
    HTTP_PID copyMyPIDx(myPIDx);
    HTTP_PID copyMyPIDpsi(myPIDpsi);
    HTTP_PID copyMyPIDphi(myPIDphi);
    long double copyRotation = rotation;
    long double copyF[2];
    copyF[0] = F[0];
    copyF[1] = F[1];

    // std::thread t([&cv, &copyMyPIDx, &copyMyPIDpsi, &copyMyPIDphi, &copyRotation, &copyF]() {
    std::thread t([&cv]()
                  {
        try {
            long double pidx_value = timedPidUpdate("x", myPIDx, myBot.xp);  // Pid over linear a speed
            long double pidpsi_value = timedPidUpdate("psi", myPIDpsi, -myBot.psip);  // Pid over psi angular speed rotation


            long double tilt = - pidx_value + myBot.phi;
            rotation = pidpsi_value;
            //copyRotation = pidpsi_value;

            long double pidphi_value = timedPidUpdate("phi", myPIDphi, tilt);  // pid over the pendulum angle phi
            //long double pidphi_value = copyMyPIDphi.update(tilt);  // pid over the pendulum angle phi

            F[0] = -pidphi_value-rotation;
            F[1] = -pidphi_value+rotation;
            //copyF[0] = -pidphi_value-copyRotation;
            //copyF[1] = -pidphi_value+copyRotation;
            // Since there is no webserver, we simulate the missed requests randomly
            /*if(!(rand()%20)) {
                std::this_thread::sleep_for(11ms);
            }*/
            cv.notify_one();
        }
        //Cetches JSON parse error, and sleeps until the timeout passes
        catch (const std::exception& error) {
            std::ostringstream message;
            message << "correction worker catch exception=\""
                    << error.what() << "\" sleep_ms=" << response_timeout;
            debugLog(message.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(response_timeout));
            //std::this_thread::sleep_for(5ms);
        }
        catch (...) {
            std::ostringstream message;
            message << "correction worker catch unknown exception sleep_ms="
                    << response_timeout;
            debugLog(message.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(response_timeout));
            //std::this_thread::sleep_for(5ms);
        } });

    t.detach();

    std::unique_lock<std::mutex> l(m);
    // if(cv.wait_for(l, 20ms) == std::cv_status::timeout) {
    if (cv.wait_for(l, std::chrono::milliseconds(response_timeout)) == std::cv_status::timeout)
    {
        std::ostringstream timeout_message;
        timeout_message << "correction wait result main_thread_timeout=true"
                        << " timeout_ms=" << response_timeout;
        debugLog(timeout_message.str());
        // t.join();
        // printf("runtime_error timeout\n");
        // throw std::runtime_error("Timeout");
        // t.join();
        std::cout << response_timeout << std::endl;
        throw std::exception();
        // throw std::runtime_error("Timeout");
    }
    debugLog("correction wait result main_thread_timeout=false");
    /*myPIDx = copyMyPIDx;
    myPIDpsi = copyMyPIDpsi;
    myPIDphi = copyMyPIDphi;
    rotation = copyRotation;
    F[0] = copyF[0];
    F[1] = copyF[1];*/
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

    HTTP_PID copyMyPIDx(myPIDx);
    HTTP_PID copyMyPIDpsi(myPIDpsi);
    HTTP_PID copyMyPIDphi(myPIDphi);
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
        debugLog("timeoutCorrection catch: restoring PID state and motor forces");
        // printf("runtime_error timeout\n");
        std::this_thread::sleep_for(10ms);
        timeout_happened = true;
        myPIDx = copyMyPIDx;
        myPIDpsi = copyMyPIDpsi;
        myPIDphi = copyMyPIDphi;
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

void animation(){
    double current_time = getElapsedTime();
    delta_time = current_time-ref_time;
    update_delta_time = current_time-update_ref_time;

    bool topple_event = false; // If robot fell over in current iteration

    if (delta_time > 1.0/FPS){
        if (myBot.phi < 0.00001 && myBot.phi > -0.00001 && myBot.phip < 0.00001 && myBot.phip > -0.00001){
            std::cout<<"Evertything is zero."<<std::endl;
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
        if (reset_if_toppled && topple_event && !toppled){
            time_toppled = current_time;
            toppled = true;
            // Log reset countdown start
            std::ostringstream reset_message;
            reset_message <<  "Robot toppled over. Resetting in " << reset_countdown_seconds << " seconds...";
            debugLog(reset_message.str());
        }
        // Reset if countdown complete
        if (reset_if_toppled && toppled && (current_time - time_toppled >= reset_countdown_seconds)) {

            // Reset robot
            initPIDs();
            myBot.phi = 0;
            myBot.phip = 0;
            toppled = false;
            topple_event = false;

            // Log reset complete
            std::ostringstream reset_done_message;
            reset_done_message <<  "Robot reset complete.";
            debugLog(reset_done_message.str());
        }

        ref_time = getElapsedTime();
        if (timeout_happened == false) update_ref_time = getElapsedTime();
        influxdbwriter.Write(myBot.phi, timeout_happened, update_delta_time);
        timeout_happened = false;
    }
}

int main(int argc, char **argv)
{
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    // influxdbwriter = InfluxDBWriter(std::string("http://influxdb.default.svc.cluster.local:8086"), std::string("robot"));
    influxdbwriter = InfluxDBWriter(std::string("http://influxdb.default.svc.cluster.local:8086"), std::string("robot"), std::string(hostname));
    // influxdbwriter = InfluxDBWriter("http://influxdb.default.svc.cluster.local:8086", "robot", hostname);
    if (argc > 2)
    {
        response_timeout = std::atoll(argv[1]);
        FPS = std::atoll(argv[2]);
    }
    const char* debug_env = std::getenv("ROBOT_DEBUG");
    if (debug_env != nullptr) {
        debug_mode = parseDebugFlag(debug_env);
    }
    if (argc > 3) {
        debug_mode = parseDebugFlag(argv[3]);
    }
    dt = 1.0f / FPS;
    // Auto reset via env var
    const char* auto_reset_env = std::getenv("RESET_COUNTDOWN_SECONDS");
    if (auto_reset_env != nullptr) {
        reset_if_toppled = true;
        reset_countdown_seconds = std::atoll(auto_reset_env);
    }
    // Auto reset via cli arg
    if (argc > 4){
        reset_if_toppled = true;
        reset_countdown_seconds = std::atoll(argv[4]);
    }
    std::ostringstream config_message;
    config_message << "debug enabled response_timeout_ms=" << response_timeout
                   << " FPS=" << FPS
                   << " dt=" << static_cast<double>(dt)
                   << " auto_reset=" << reset_if_toppled;
    if (reset_if_toppled) {
        config_message << " reset_countdown_seconds=" << reset_countdown_seconds;
    }
    debugLog(config_message.str());
    // std::thread correctionThread(threadCorrection);
    srand((unsigned)time(0));
    initPIDs();
    while (1){
        animation();
    }
}
