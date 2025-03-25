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

class GPIOMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPIOMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPIOMode
    let len;
    let data = new GPIOMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GPIOMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67dcb56ddb7c7d3c3c961e401c8325a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 GPIO_MODE_UNSPECIFIED = 0
    
    uint32 GPIO_MODE_INPUT_FLOATING = 1
    
    uint32 GPIO_MODE_OUTPUT_PUSH_PULL = 2
    
    uint32 GPIO_MODE_OUTPUT_OPEN_DRAIN = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPIOMode(null);
    return resolved;
    }
};

// Constants for message
GPIOMode.Constants = {
  GPIO_MODE_UNSPECIFIED: 0,
  GPIO_MODE_INPUT_FLOATING: 1,
  GPIO_MODE_OUTPUT_PUSH_PULL: 2,
  GPIO_MODE_OUTPUT_OPEN_DRAIN: 3,
}

module.exports = GPIOMode;
