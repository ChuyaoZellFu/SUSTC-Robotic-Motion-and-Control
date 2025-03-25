// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class FactoryEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FactoryEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FactoryEvent
    let len;
    let data = new FactoryEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FactoryEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f21e922380f9063597fbdefcc7ac55fa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_FACTORY_EVENT = 0
    
    uint32 FACTORY_DEFAULT_RESTORED = 1
    
    uint32 NETWORK_FACTORY_DEFAULT_RESTORED = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FactoryEvent(null);
    return resolved;
    }
};

// Constants for message
FactoryEvent.Constants = {
  UNSPECIFIED_FACTORY_EVENT: 0,
  FACTORY_DEFAULT_RESTORED: 1,
  NETWORK_FACTORY_DEFAULT_RESTORED: 2,
}

module.exports = FactoryEvent;
