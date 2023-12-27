// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/Eulers
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
#include "ardvarc/Eulers.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_Eulers_common : public MATLABROSMsgInterface<ardvarc::Eulers> {
  public:
    virtual ~ardvarc_msg_Eulers_common(){}
    virtual void copy_from_struct(ardvarc::Eulers* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::Eulers* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_Eulers_common::copy_from_struct(ardvarc::Eulers* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //eulers
        const matlab::data::TypedArray<double> eulers_arr = arr["Eulers_"];
        size_t nelem = 3;
        	std::copy(eulers_arr.begin(), eulers_arr.begin()+nelem, msg->eulers.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Eulers_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Eulers_' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_Eulers_common::get_arr(MDFactory_T& factory, const ardvarc::Eulers* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Eulers_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/Eulers");
    // eulers
    auto currentElement_eulers = (msg + ctr)->eulers;
    outArray[ctr]["Eulers_"] = factory.createArray<ardvarc::Eulers::_eulers_type::const_iterator, double>({currentElement_eulers.size(),1}, currentElement_eulers.begin(), currentElement_eulers.end());
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_Eulers_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_Eulers_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_Eulers_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::Eulers,ardvarc_msg_Eulers_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_Eulers_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::Eulers,ardvarc::Eulers::ConstPtr,ardvarc_msg_Eulers_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_Eulers_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::Eulers,ardvarc_msg_Eulers_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_Eulers_common, MATLABROSMsgInterface<ardvarc::Eulers>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_Eulers_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1