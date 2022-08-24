// Generated by gencpp from file ros_exercises/compute_statistics.msg
// DO NOT EDIT!


#ifndef ROS_EXERCISES_MESSAGE_COMPUTE_STATISTICS_H
#define ROS_EXERCISES_MESSAGE_COMPUTE_STATISTICS_H

#include <ros/service_traits.h>


#include <ros_exercises/compute_statisticsRequest.h>
#include <ros_exercises/compute_statisticsResponse.h>


namespace ros_exercises
{

struct compute_statistics
{

typedef compute_statisticsRequest Request;
typedef compute_statisticsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct compute_statistics
} // namespace ros_exercises


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ros_exercises::compute_statistics > {
  static const char* value()
  {
    return "89a0f83e0765b462301612617aaed4ac";
  }

  static const char* value(const ::ros_exercises::compute_statistics&) { return value(); }
};

template<>
struct DataType< ::ros_exercises::compute_statistics > {
  static const char* value()
  {
    return "ros_exercises/compute_statistics";
  }

  static const char* value(const ::ros_exercises::compute_statistics&) { return value(); }
};


// service_traits::MD5Sum< ::ros_exercises::compute_statisticsRequest> should match
// service_traits::MD5Sum< ::ros_exercises::compute_statistics >
template<>
struct MD5Sum< ::ros_exercises::compute_statisticsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ros_exercises::compute_statistics >::value();
  }
  static const char* value(const ::ros_exercises::compute_statisticsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_exercises::compute_statisticsRequest> should match
// service_traits::DataType< ::ros_exercises::compute_statistics >
template<>
struct DataType< ::ros_exercises::compute_statisticsRequest>
{
  static const char* value()
  {
    return DataType< ::ros_exercises::compute_statistics >::value();
  }
  static const char* value(const ::ros_exercises::compute_statisticsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ros_exercises::compute_statisticsResponse> should match
// service_traits::MD5Sum< ::ros_exercises::compute_statistics >
template<>
struct MD5Sum< ::ros_exercises::compute_statisticsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ros_exercises::compute_statistics >::value();
  }
  static const char* value(const ::ros_exercises::compute_statisticsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ros_exercises::compute_statisticsResponse> should match
// service_traits::DataType< ::ros_exercises::compute_statistics >
template<>
struct DataType< ::ros_exercises::compute_statisticsResponse>
{
  static const char* value()
  {
    return DataType< ::ros_exercises::compute_statistics >::value();
  }
  static const char* value(const ::ros_exercises::compute_statisticsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROS_EXERCISES_MESSAGE_COMPUTE_STATISTICS_H
