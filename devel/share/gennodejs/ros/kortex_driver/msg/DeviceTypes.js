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

class DeviceTypes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceTypes
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceTypes
    let len;
    let data = new DeviceTypes(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceTypes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46c218f31055194b4054a2785d94d7f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_DEVICE_TYPE = 0
    
    uint32 BASE = 1
    
    uint32 VISION = 2
    
    uint32 BIG_ACTUATOR = 3
    
    uint32 SMALL_ACTUATOR = 4
    
    uint32 INTERCONNECT = 5
    
    uint32 GRIPPER = 6
    
    uint32 MEDIUM_ACTUATOR = 7
    
    uint32 XBIG_ACTUATOR = 8
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceTypes(null);
    return resolved;
    }
};

// Constants for message
DeviceTypes.Constants = {
  UNSPECIFIED_DEVICE_TYPE: 0,
  BASE: 1,
  VISION: 2,
  BIG_ACTUATOR: 3,
  SMALL_ACTUATOR: 4,
  INTERCONNECT: 5,
  GRIPPER: 6,
  MEDIUM_ACTUATOR: 7,
  XBIG_ACTUATOR: 8,
}

module.exports = DeviceTypes;
