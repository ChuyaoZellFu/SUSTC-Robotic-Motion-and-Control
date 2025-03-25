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

class SignalQuality {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SignalQuality
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SignalQuality
    let len;
    let data = new SignalQuality(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SignalQuality';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '080b7090353f86dd3efade43f40ad044';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_SIGNAL_QUALITY = 0
    
    uint32 POOR = 1
    
    uint32 FAIR = 2
    
    uint32 GOOD = 3
    
    uint32 EXCELLENT = 4
    
    uint32 NONE = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SignalQuality(null);
    return resolved;
    }
};

// Constants for message
SignalQuality.Constants = {
  UNSPECIFIED_SIGNAL_QUALITY: 0,
  POOR: 1,
  FAIR: 2,
  GOOD: 3,
  EXCELLENT: 4,
  NONE: 5,
}

module.exports = SignalQuality;
