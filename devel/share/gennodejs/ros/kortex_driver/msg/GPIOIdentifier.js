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

class GPIOIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPIOIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPIOIdentifier
    let len;
    let data = new GPIOIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GPIOIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d6dda494f04fdbc115c7ef311e5d45d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 GPIO_IDENTIFIER_UNSPECIFIED = 0
    
    uint32 GPIO_IDENTIFIER_1 = 1
    
    uint32 GPIO_IDENTIFIER_2 = 2
    
    uint32 GPIO_IDENTIFIER_3 = 3
    
    uint32 GPIO_IDENTIFIER_4 = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPIOIdentifier(null);
    return resolved;
    }
};

// Constants for message
GPIOIdentifier.Constants = {
  GPIO_IDENTIFIER_UNSPECIFIED: 0,
  GPIO_IDENTIFIER_1: 1,
  GPIO_IDENTIFIER_2: 2,
  GPIO_IDENTIFIER_3: 3,
  GPIO_IDENTIFIER_4: 4,
}

module.exports = GPIOIdentifier;
