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

class CartesianReferenceFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianReferenceFrame
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianReferenceFrame
    let len;
    let data = new CartesianReferenceFrame(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianReferenceFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82b14912eebdea49eac0f3e20689bea8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 CARTESIAN_REFERENCE_FRAME_UNSPECIFIED = 0
    
    uint32 CARTESIAN_REFERENCE_FRAME_MIXED = 1
    
    uint32 CARTESIAN_REFERENCE_FRAME_TOOL = 2
    
    uint32 CARTESIAN_REFERENCE_FRAME_BASE = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianReferenceFrame(null);
    return resolved;
    }
};

// Constants for message
CartesianReferenceFrame.Constants = {
  CARTESIAN_REFERENCE_FRAME_UNSPECIFIED: 0,
  CARTESIAN_REFERENCE_FRAME_MIXED: 1,
  CARTESIAN_REFERENCE_FRAME_TOOL: 2,
  CARTESIAN_REFERENCE_FRAME_BASE: 3,
}

module.exports = CartesianReferenceFrame;
