#ifndef INFLUXDBWRITER
#define INFLUXDBWRITER

#include <iostream>
#include <string>
#include <InfluxDBFactory.h>

struct InfluxDBWriter {
    //influxdb::InfluxDB* db;
    std::unique_ptr<influxdb::InfluxDB> db;
    std::string influx_address;
    std::string db_name;
    std::string robotname;
    InfluxDBWriter() {}
    //InfluxDBWriter(std::string influx_address, std::string db_name, std::string robot_name) : influx_address(influx_address), db_name(db_name), robotname(robotname) {
    InfluxDBWriter(std::string influx_address, std::string db_name, std::string robotname) {
        this->robotname = robotname;
        std::string db_connection_string = influx_address + "?db=" + db_name;
        db = influxdb::InfluxDBFactory::Get(db_connection_string.c_str());
        db->createDatabaseIfNotExists();
    }

    //TODO: make it nice....
    void WriteAngle(float angle, bool timeout_happened, float delta_time) {
        db->write(influxdb::Point{"angle"}.addTag("robotname",robotname).addField("timeout", timeout_happened ? 1 : 0).addField("value",angle).addField("delta_time",delta_time));
    }
    void WriteRtt(float rtt, bool timeout_happened, float delta_time){
        db->write(influxdb::Point{"rtt"}.addTag("robotname",robotname).addField("timeout", timeout_happened ? 1 : 0).addField("value",angle).addField("delta_time",delta_time));
    }
};

#endif

