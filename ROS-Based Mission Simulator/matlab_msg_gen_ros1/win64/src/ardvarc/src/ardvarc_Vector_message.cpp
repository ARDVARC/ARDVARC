// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ardvarc/Vector
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
#include "ardvarc/Vector.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class ARDVARC_EXPORT ardvarc_msg_Vector_common : public MATLABROSMsgInterface<ardvarc::Vector> {
  public:
    virtual ~ardvarc_msg_Vector_common(){}
    virtual void copy_from_struct(ardvarc::Vector* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ardvarc::Vector* msg, MultiLibLoader loader, size_t size = 1);
};
  void ardvarc_msg_Vector_common::copy_from_struct(ardvarc::Vector* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //vector
        const matlab::data::TypedArray<double> vector_arr = arr["Vector_"];
        size_t nelem = 3;
        	std::copy(vector_arr.begin(), vector_arr.begin()+nelem, msg->vector.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Vector_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Vector_' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ardvarc_msg_Vector_common::get_arr(MDFactory_T& factory, const ardvarc::Vector* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Vector_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ardvarc/Vector");
    // vector
    auto currentElement_vector = (msg + ctr)->vector;
    outArray[ctr]["Vector_"] = factory.createArray<ardvarc::Vector::_vector_type::const_iterator, double>({currentElement_vector.size(),1}, currentElement_vector.begin(), currentElement_vector.end());
    }
    return std::move(outArray);
  } 
class ARDVARC_EXPORT ardvarc_Vector_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ardvarc_Vector_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ardvarc_Vector_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ardvarc::Vector,ardvarc_msg_Vector_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ardvarc_Vector_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ardvarc::Vector,ardvarc::Vector::ConstPtr,ardvarc_msg_Vector_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ardvarc_Vector_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ardvarc::Vector,ardvarc_msg_Vector_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ardvarc_msg_Vector_common, MATLABROSMsgInterface<ardvarc::Vector>)
CLASS_LOADER_REGISTER_CLASS(ardvarc_Vector_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1