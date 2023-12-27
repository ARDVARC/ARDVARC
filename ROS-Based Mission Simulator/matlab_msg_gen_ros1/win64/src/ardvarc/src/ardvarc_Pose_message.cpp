// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/Pose
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
#include "ardvarc/Pose.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_Pose_common : public MATLABROSMsgInterface<ardvarc::Pose> {
  public:
    virtual ~ardvarc_msg_Pose_common(){}
    virtual void copy_from_struct(ardvarc::Pose* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::Pose* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_Pose_common::copy_from_struct(ardvarc::Pose* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //position
        const matlab::data::StructArray position_arr = arr["Position"];
        auto msgClassPtr_position = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
        msgClassPtr_position->copy_from_struct(&msg->position,position_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Position' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Position' is wrong type; expected a struct.");
    }
    try {
        //eulers
        const matlab::data::StructArray eulers_arr = arr["Eulers"];
        auto msgClassPtr_eulers = getCommonObject<ardvarc::Eulers>("ardvarc_msg_Eulers_common",loader);
        msgClassPtr_eulers->copy_from_struct(&msg->eulers,eulers_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Eulers' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Eulers' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_Pose_common::get_arr(MDFactory_T& factory, const ardvarc::Pose* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Position","Eulers"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/Pose");
    // position
    auto currentElement_position = (msg + ctr)->position;
    auto msgClassPtr_position = getCommonObject<ardvarc::Point>("ardvarc_msg_Point_common",loader);
    outArray[ctr]["Position"] = msgClassPtr_position->get_arr(factory, &currentElement_position, loader);
    // eulers
    auto currentElement_eulers = (msg + ctr)->eulers;
    auto msgClassPtr_eulers = getCommonObject<ardvarc::Eulers>("ardvarc_msg_Eulers_common",loader);
    outArray[ctr]["Eulers"] = msgClassPtr_eulers->get_arr(factory, &currentElement_eulers, loader);
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_Pose_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_Pose_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_Pose_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::Pose,ardvarc_msg_Pose_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_Pose_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::Pose,ardvarc::Pose::ConstPtr,ardvarc_msg_Pose_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_Pose_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::Pose,ardvarc_msg_Pose_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_Pose_common, MATLABROSMsgInterface<ardvarc::Pose>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_Pose_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1