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

class GPIOPull {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPIOPull
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPIOPull
    let len;
    let data = new GPIOPull(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GPIOPull';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '76d74f4a9f22d9530bd83becdc56fba5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 GPIO_PULL_UNSPECIFIED = 0
    
    uint32 GPIO_PULL_NONE = 1
    
    uint32 GPIO_PULL_UP = 2
    
    uint32 GPIO_PULL_DOWN = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPIOPull(null);
    return resolved;
    }
};

// Constants for message
GPIOPull.Constants = {
  GPIO_PULL_UNSPECIFIED: 0,
  GPIO_PULL_NONE: 1,
  GPIO_PULL_UP: 2,
  GPIO_PULL_DOWN: 3,
}

module.exports = GPIOPull;
