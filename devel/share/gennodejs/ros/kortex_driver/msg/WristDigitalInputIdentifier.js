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

class WristDigitalInputIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WristDigitalInputIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WristDigitalInputIdentifier
    let len;
    let data = new WristDigitalInputIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WristDigitalInputIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ca46a2da6d637ba3dfa3b30f7d69da3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_WRIST_DIGITAL = 0
    
    uint32 WRIST_BUTTON_1 = 1
    
    uint32 WRIST_BUTTON_2 = 2
    
    uint32 WRIST_BUTTON_BOTH = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WristDigitalInputIdentifier(null);
    return resolved;
    }
};

// Constants for message
WristDigitalInputIdentifier.Constants = {
  UNSPECIFIED_WRIST_DIGITAL: 0,
  WRIST_BUTTON_1: 1,
  WRIST_BUTTON_2: 2,
  WRIST_BUTTON_BOTH: 3,
}

module.exports = WristDigitalInputIdentifier;
