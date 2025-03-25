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

class ModelId {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ModelId
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ModelId
    let len;
    let data = new ModelId(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ModelId';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7fb7d718b4a478fa436cc49becd077b0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 MODEL_ID_UNSPECIFIED = 0
    
    uint32 MODEL_ID_L53 = 1
    
    uint32 MODEL_ID_L31 = 2
    
    uint32 MODEL_ID_HDK = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ModelId(null);
    return resolved;
    }
};

// Constants for message
ModelId.Constants = {
  MODEL_ID_UNSPECIFIED: 0,
  MODEL_ID_L53: 1,
  MODEL_ID_L31: 2,
  MODEL_ID_HDK: 3,
}

module.exports = ModelId;
