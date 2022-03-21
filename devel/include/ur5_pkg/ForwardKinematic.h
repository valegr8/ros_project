// Generated by gencpp from file ur5_pkg/ForwardKinematic.msg
// DO NOT EDIT!


#ifndef UR5_PKG_MESSAGE_FORWARDKINEMATIC_H
#define UR5_PKG_MESSAGE_FORWARDKINEMATIC_H

#include <ros/service_traits.h>


#include <ur5_pkg/ForwardKinematicRequest.h>
#include <ur5_pkg/ForwardKinematicResponse.h>


namespace ur5_pkg
{

struct ForwardKinematic
{

typedef ForwardKinematicRequest Request;
typedef ForwardKinematicResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ForwardKinematic
} // namespace ur5_pkg


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ur5_pkg::ForwardKinematic > {
  static const char* value()
  {
    return "1d89c8f7ccd238d24ca1c2a794f9dbfe";
  }

  static const char* value(const ::ur5_pkg::ForwardKinematic&) { return value(); }
};

template<>
struct DataType< ::ur5_pkg::ForwardKinematic > {
  static const char* value()
  {
    return "ur5_pkg/ForwardKinematic";
  }

  static const char* value(const ::ur5_pkg::ForwardKinematic&) { return value(); }
};


// service_traits::MD5Sum< ::ur5_pkg::ForwardKinematicRequest> should match
// service_traits::MD5Sum< ::ur5_pkg::ForwardKinematic >
template<>
struct MD5Sum< ::ur5_pkg::ForwardKinematicRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ur5_pkg::ForwardKinematic >::value();
  }
  static const char* value(const ::ur5_pkg::ForwardKinematicRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ur5_pkg::ForwardKinematicRequest> should match
// service_traits::DataType< ::ur5_pkg::ForwardKinematic >
template<>
struct DataType< ::ur5_pkg::ForwardKinematicRequest>
{
  static const char* value()
  {
    return DataType< ::ur5_pkg::ForwardKinematic >::value();
  }
  static const char* value(const ::ur5_pkg::ForwardKinematicRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ur5_pkg::ForwardKinematicResponse> should match
// service_traits::MD5Sum< ::ur5_pkg::ForwardKinematic >
template<>
struct MD5Sum< ::ur5_pkg::ForwardKinematicResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ur5_pkg::ForwardKinematic >::value();
  }
  static const char* value(const ::ur5_pkg::ForwardKinematicResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ur5_pkg::ForwardKinematicResponse> should match
// service_traits::DataType< ::ur5_pkg::ForwardKinematic >
template<>
struct DataType< ::ur5_pkg::ForwardKinematicResponse>
{
  static const char* value()
  {
    return DataType< ::ur5_pkg::ForwardKinematic >::value();
  }
  static const char* value(const ::ur5_pkg::ForwardKinematicResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // UR5_PKG_MESSAGE_FORWARDKINEMATIC_H
