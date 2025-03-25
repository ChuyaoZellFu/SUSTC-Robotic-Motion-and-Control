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

class ErrorCodes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ErrorCodes
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ErrorCodes
    let len;
    let data = new ErrorCodes(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ErrorCodes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cb78b6e4994aaf216b83b68ecaa14cbd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ERROR_NONE = 0
    
    uint32 ERROR_PROTOCOL_SERVER = 1
    
    uint32 ERROR_PROTOCOL_CLIENT = 2
    
    uint32 ERROR_DEVICE = 3
    
    uint32 ERROR_INTERNAL = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ErrorCodes(null);
    return resolved;
    }
};

// Constants for message
ErrorCodes.Constants = {
  ERROR_NONE: 0,
  ERROR_PROTOCOL_SERVER: 1,
  ERROR_PROTOCOL_CLIENT: 2,
  ERROR_DEVICE: 3,
  ERROR_INTERNAL: 4,
}

module.exports = ErrorCodes;
