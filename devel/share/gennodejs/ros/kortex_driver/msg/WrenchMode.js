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

class WrenchMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WrenchMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WrenchMode
    let len;
    let data = new WrenchMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WrenchMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21f4098a76f681f49eb3f39d12684c72';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_WRENCH_MODE = 0
    
    uint32 WRENCH_RESTRICTED = 1
    
    uint32 WRENCH_NORMAL = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WrenchMode(null);
    return resolved;
    }
};

// Constants for message
WrenchMode.Constants = {
  UNSPECIFIED_WRENCH_MODE: 0,
  WRENCH_RESTRICTED: 1,
  WRENCH_NORMAL: 2,
}

module.exports = WrenchMode;
