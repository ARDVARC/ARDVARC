// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/TestArrayMsg
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
#include "ardvarc/TestArrayMsg.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_TestArrayMsg_common : public MATLABROSMsgInterface<ardvarc::TestArrayMsg> {
  public:
    virtual ~ardvarc_msg_TestArrayMsg_common(){}
    virtual void copy_from_struct(ardvarc::TestArrayMsg* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::TestArrayMsg* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_TestArrayMsg_common::copy_from_struct(ardvarc::TestArrayMsg* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //vec_tor
        const matlab::data::TypedArray<double> vec_tor_arr = arr["VecTor"];
        size_t nelem = 3;
        	std::copy(vec_tor_arr.begin(), vec_tor_arr.begin()+nelem, msg->vec_tor.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'VecTor' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'VecTor' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_TestArrayMsg_common::get_arr(MDFactory_T& factory, const ardvarc::TestArrayMsg* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","VecTor"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/TestArrayMsg");
    // vec_tor
    auto currentElement_vec_tor = (msg + ctr)->vec_tor;
    outArray[ctr]["VecTor"] = factory.createArray<ardvarc::TestArrayMsg::_vec_tor_type::const_iterator, double>({currentElement_vec_tor.size(),1}, currentElement_vec_tor.begin(), currentElement_vec_tor.end());
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_TestArrayMsg_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_TestArrayMsg_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_TestArrayMsg_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::TestArrayMsg,ardvarc_msg_TestArrayMsg_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_TestArrayMsg_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::TestArrayMsg,ardvarc::TestArrayMsg::ConstPtr,ardvarc_msg_TestArrayMsg_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_TestArrayMsg_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::TestArrayMsg,ardvarc_msg_TestArrayMsg_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_TestArrayMsg_common, MATLABROSMsgInterface<ardvarc::TestArrayMsg>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_TestArrayMsg_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1