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

class CartesianReferenceFrameInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reference_frame = null;
    }
    else {
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianReferenceFrameInfo
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.uint32(obj.reference_frame, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianReferenceFrameInfo
    let len;
    let data = new CartesianReferenceFrameInfo(null);
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianReferenceFrameInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6e1a124b6ab31faa150246edd7907de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 reference_frame
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianReferenceFrameInfo(null);
    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = 0
    }

    return resolved;
    }
};

module.exports = CartesianReferenceFrameInfo;
