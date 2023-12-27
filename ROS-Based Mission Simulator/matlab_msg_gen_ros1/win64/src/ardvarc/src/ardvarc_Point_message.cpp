// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/Point
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
#include "ardvarc/Point.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_Point_common : public MATLABROSMsgInterface<ardvarc::Point> {
  public:
    virtual ~ardvarc_msg_Point_common(){}
    virtual void copy_from_struct(ardvarc::Point* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::Point* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_Point_common::copy_from_struct(ardvarc::Point* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //point
        const matlab::data::TypedArray<double> point_arr = arr["Point_"];
        size_t nelem = 3;
        	std::copy(point_arr.begin(), point_arr.begin()+nelem, msg->point.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Point_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Point_' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_Point_common::get_arr(MDFactory_T& factory, const ardvarc::Point* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Point_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/Point");
    // point
    auto currentElement_point = (msg + ctr)->point;
    outArray[ctr]["Point_"] = factory.createArray<ardvarc::Point::_point_type::const_iterator, double>({currentElement_point.size(),1}, currentElement_point.begin(), currentElement_point.end());
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_Point_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_Point_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_Point_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::Point,ardvarc_msg_Point_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_Point_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::Point,ardvarc::Point::ConstPtr,ardvarc_msg_Point_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_Point_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::Point,ardvarc_msg_Point_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_Point_common, MATLABROSMsgInterface<ardvarc::Point>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_Point_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1