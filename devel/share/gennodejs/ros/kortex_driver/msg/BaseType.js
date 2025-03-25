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

class BaseType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseType
    let len;
    let data = new BaseType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BaseType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7e0a5914d628ae4f54d3bdfb80a2c377';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 BASE_TYPE_UNSPECIFIED = 0
    
    uint32 BASE_TYPE_L53_QUICK_CONNECT = 1
    
    uint32 BASE_TYPE_L53_FIXED = 2
    
    uint32 BASE_TYPE_L31_FIXED = 3
    
    uint32 BASE_TYPE_HDK_FIXED = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BaseType(null);
    return resolved;
    }
};

// Constants for message
BaseType.Constants = {
  BASE_TYPE_UNSPECIFIED: 0,
  BASE_TYPE_L53_QUICK_CONNECT: 1,
  BASE_TYPE_L53_FIXED: 2,
  BASE_TYPE_L31_FIXED: 3,
  BASE_TYPE_HDK_FIXED: 4,
}

module.exports = BaseType;
