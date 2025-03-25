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

class GPIOValue {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPIOValue
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPIOValue
    let len;
    let data = new GPIOValue(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GPIOValue';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2450504e1dcf956a73a1061ef8b94f94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 GPIO_VALUE_UNSPECIFIED = 0
    
    uint32 GPIO_VALUE_LOW = 1
    
    uint32 GPIO_VALUE_HIGH = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPIOValue(null);
    return resolved;
    }
};

// Constants for message
GPIOValue.Constants = {
  GPIO_VALUE_UNSPECIFIED: 0,
  GPIO_VALUE_LOW: 1,
  GPIO_VALUE_HIGH: 2,
}

module.exports = GPIOValue;
