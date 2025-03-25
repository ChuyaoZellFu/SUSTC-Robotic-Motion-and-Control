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

class InterfaceModuleType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterfaceModuleType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterfaceModuleType
    let len;
    let data = new InterfaceModuleType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterfaceModuleType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f61dc75115f701d8cd0d9b3951030045';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 INTERFACE_MODULE_TYPE_UNSPECIFIED = 0
    
    uint32 INTERFACE_MODULE_TYPE_NOT_INSTALLED = 1
    
    uint32 INTERFACE_MODULE_TYPE_L53 = 2
    
    uint32 INTERFACE_MODULE_TYPE_L31 = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InterfaceModuleType(null);
    return resolved;
    }
};

// Constants for message
InterfaceModuleType.Constants = {
  INTERFACE_MODULE_TYPE_UNSPECIFIED: 0,
  INTERFACE_MODULE_TYPE_NOT_INSTALLED: 1,
  INTERFACE_MODULE_TYPE_L53: 2,
  INTERFACE_MODULE_TYPE_L31: 3,
}

module.exports = InterfaceModuleType;
