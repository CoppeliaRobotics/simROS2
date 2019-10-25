#ifndef SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H
#define SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H

#include <rclcpp/rclcpp.hpp>

struct ReadOptions
{
    bool uint8array_as_string;
    ReadOptions();
};

struct WriteOptions
{
    bool uint8array_as_string;
    WriteOptions();
};

void read__bool(int stack, bool *value, const ReadOptions *opt = NULL);
void read__byte(int stack, uint8_t *value, const ReadOptions *opt = NULL);
void read__char(int stack, unsigned char *value, const ReadOptions *opt = NULL);
void read__int8(int stack, int8_t *value, const ReadOptions *opt = NULL);
void read__uint8(int stack, uint8_t *value, const ReadOptions *opt = NULL);
void read__int16(int stack, int16_t *value, const ReadOptions *opt = NULL);
void read__uint16(int stack, uint16_t *value, const ReadOptions *opt = NULL);
void read__int32(int stack, int32_t *value, const ReadOptions *opt = NULL);
void read__uint32(int stack, uint32_t *value, const ReadOptions *opt = NULL);
void read__int64(int stack, int64_t *value, const ReadOptions *opt = NULL);
void read__uint64(int stack, uint64_t *value, const ReadOptions *opt = NULL);
void read__float32(int stack, float *value, const ReadOptions *opt = NULL);
void read__float64(int stack, double *value, const ReadOptions *opt = NULL);
void read__string(int stack, std::string *value, const ReadOptions *opt = NULL);
void read__time(int stack, rclcpp::Time *value, const ReadOptions *opt = NULL);
void read__duration(int stack, rclcpp::Duration *value, const ReadOptions *opt = NULL);
void write__bool(bool value, int stack, const WriteOptions *opt = NULL);
void write__byte(uint8_t value, int stack, const WriteOptions *opt = NULL);
void write__char(unsigned char value, int stack, const WriteOptions *opt = NULL);
void write__int8(int8_t value, int stack, const WriteOptions *opt = NULL);
void write__uint8(uint8_t value, int stack, const WriteOptions *opt = NULL);
void write__int16(int16_t value, int stack, const WriteOptions *opt = NULL);
void write__uint16(uint16_t value, int stack, const WriteOptions *opt = NULL);
void write__int32(int32_t value, int stack, const WriteOptions *opt = NULL);
void write__uint32(uint32_t value, int stack, const WriteOptions *opt = NULL);
void write__int64(int64_t value, int stack, const WriteOptions *opt = NULL);
void write__uint64(uint64_t value, int stack, const WriteOptions *opt = NULL);
void write__float32(float value, int stack, const WriteOptions *opt = NULL);
void write__float64(double value, int stack, const WriteOptions *opt = NULL);
void write__string(std::string value, int stack, const WriteOptions *opt = NULL);
void write__time(rclcpp::Time value, int stack, const WriteOptions *opt = NULL);
void write__duration(rclcpp::Duration value, int stack, const WriteOptions *opt = NULL);

#endif // SIM_ROS2_PLUGIN__ROS_MSG_BUILTIN_IO__H
