// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/FlightPlan
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "ros/ros.h"
#include "ardvarc/FlightPlan.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_FlightPlan_common : public MATLABROSMsgInterface<ardvarc::FlightPlan> {
  public:
    virtual ~ardvarc_msg_FlightPlan_common(){}
    virtual void copy_from_struct(ardvarc::FlightPlan* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::FlightPlan* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_FlightPlan_common::copy_from_struct(ardvarc::FlightPlan* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //go_to_center
        const matlab::data::StructArray go_to_center_arr = arr["GoToCenter"];
        auto msgClassPtr_go_to_center = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
        msgClassPtr_go_to_center->copy_from_struct(&msg->go_to_center,go_to_center_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'GoToCenter' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'GoToCenter' is wrong type; expected a struct.");
    }
    try {
        //orbit_radius
        const matlab::data::TypedArray<double> orbit_radius_arr = arr["OrbitRadius"];
        msg->orbit_radius = orbit_radius_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'OrbitRadius' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'OrbitRadius' is wrong type; expected a double.");
    }
    try {
        //look_at
        const matlab::data::StructArray look_at_arr = arr["LookAt"];
        auto msgClassPtr_look_at = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
        msgClassPtr_look_at->copy_from_struct(&msg->look_at,look_at_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'LookAt' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'LookAt' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_FlightPlan_common::get_arr(MDFactory_T& factory, const ardvarc::FlightPlan* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","GoToCenter","OrbitRadius","LookAt"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/FlightPlan");
    // go_to_center
    auto currentElement_go_to_center = (msg + ctr)->go_to_center;
    auto msgClassPtr_go_to_center = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
    outArray[ctr]["GoToCenter"] = msgClassPtr_go_to_center->get_arr(factory, &currentElement_go_to_center, loader);
    // orbit_radius
    auto currentElement_orbit_radius = (msg + ctr)->orbit_radius;
    outArray[ctr]["OrbitRadius"] = factory.createScalar(currentElement_orbit_radius);
    // look_at
    auto currentElement_look_at = (msg + ctr)->look_at;
    auto msgClassPtr_look_at = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
    outArray[ctr]["LookAt"] = msgClassPtr_look_at->get_arr(factory, &currentElement_look_at, loader);
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_FlightPlan_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_FlightPlan_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_FlightPlan_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::FlightPlan,ardvarc_msg_FlightPlan_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_FlightPlan_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::FlightPlan,ardvarc::FlightPlan::ConstPtr,ardvarc_msg_FlightPlan_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_FlightPlan_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::FlightPlan,ardvarc_msg_FlightPlan_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_FlightPlan_common, MATLABROSMsgInterface<ardvarc::FlightPlan>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_FlightPlan_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1