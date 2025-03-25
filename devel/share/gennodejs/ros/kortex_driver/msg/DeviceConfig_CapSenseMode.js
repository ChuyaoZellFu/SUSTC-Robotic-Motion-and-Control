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

class DeviceConfig_CapSenseMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceConfig_CapSenseMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_CapSenseMode
    let len;
    let data = new DeviceConfig_CapSenseMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceConfig_CapSenseMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab37ae8a57a132b00bf1d50bed534551';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 RESERVED = 0
    
    uint32 INACTIVE = 1
    
    uint32 ACTIVE_AUTO_THRESHOLD = 2
    
    uint32 ACTIVE_NOISE_ATT = 4
    
    uint32 ACTIVE_NORMAL = 5
    
    uint32 CONFIGURATION = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceConfig_CapSenseMode(null);
    return resolved;
    }
};

// Constants for message
DeviceConfig_CapSenseMode.Constants = {
  RESERVED: 0,
  INACTIVE: 1,
  ACTIVE_AUTO_THRESHOLD: 2,
  ACTIVE_NOISE_ATT: 4,
  ACTIVE_NORMAL: 5,
  CONFIGURATION: 6,
}

module.exports = DeviceConfig_CapSenseMode;
