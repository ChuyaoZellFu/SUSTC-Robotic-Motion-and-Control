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

class AxisOffsets {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.absolute_offset = null;
      this.relative_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('absolute_offset')) {
        this.absolute_offset = initObj.absolute_offset
      }
      else {
        this.absolute_offset = 0.0;
      }
      if (initObj.hasOwnProperty('relative_offset')) {
        this.relative_offset = initObj.relative_offset
      }
      else {
        this.relative_offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AxisOffsets
    // Serialize message field [absolute_offset]
    bufferOffset = _serializer.float32(obj.absolute_offset, buffer, bufferOffset);
    // Serialize message field [relative_offset]
    bufferOffset = _serializer.float32(obj.relative_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AxisOffsets
    let len;
    let data = new AxisOffsets(null);
    // Deserialize message field [absolute_offset]
    data.absolute_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [relative_offset]
    data.relative_offset = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/AxisOffsets';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2cf10c1327fbdc40ecc0a83625568f5b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 absolute_offset
    float32 relative_offset
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AxisOffsets(null);
    if (msg.absolute_offset !== undefined) {
      resolved.absolute_offset = msg.absolute_offset;
    }
    else {
      resolved.absolute_offset = 0.0
    }

    if (msg.relative_offset !== undefined) {
      resolved.relative_offset = msg.relative_offset;
    }
    else {
      resolved.relative_offset = 0.0
    }

    return resolved;
    }
};

module.exports = AxisOffsets;
