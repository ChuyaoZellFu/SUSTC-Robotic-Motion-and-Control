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

class WristType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WristType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WristType
    let len;
    let data = new WristType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WristType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dce76d4e649d6e568f93c37bf10d6238';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 WRIST_TYPE_UNSPECIFIED = 0
    
    uint32 WRIST_TYPE_NOT_APPLICABLE = 1
    
    uint32 WRIST_TYPE_SPHERICAL = 2
    
    uint32 WRIST_TYPE_CURVED = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WristType(null);
    return resolved;
    }
};

// Constants for message
WristType.Constants = {
  WRIST_TYPE_UNSPECIFIED: 0,
  WRIST_TYPE_NOT_APPLICABLE: 1,
  WRIST_TYPE_SPHERICAL: 2,
  WRIST_TYPE_CURVED: 3,
}

module.exports = WristType;
