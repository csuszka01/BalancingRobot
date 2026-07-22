FROM influxtmp
WORKDIR cppapp
COPY webclient.cpp .
COPY pid.cpp .
COPY httplib.h .
COPY logger.h .
COPY http_pid_set.cpp .
COPY ibalancingbot.cpp .
COPY influxdbwriter.cpp .
COPY json.hpp .
COPY main.cpp .
RUN g++ -std=c++17 -o myapp main.cpp -I/usr/local/include/InfluxDB -L/usr/local/lib -lInfluxDB -lcurl -pthread -g
RUN ldconfig
CMD "./myapp"
