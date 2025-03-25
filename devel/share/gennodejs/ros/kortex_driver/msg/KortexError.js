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

class KortexError {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.description = null;
      this.code = null;
      this.subCode = null;
    }
    else {
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('subCode')) {
        this.subCode = initObj.subCode
      }
      else {
        this.subCode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KortexError
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.uint32(obj.code, buffer, bufferOffset);
    // Serialize message field [subCode]
    bufferOffset = _serializer.uint32(obj.subCode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KortexError
    let len;
    let data = new KortexError(null);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [subCode]
    data.subCode = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.description);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/KortexError';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e30b4b6c4d6da8786415fb734a7dea1b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string description
    uint32 code
    uint32 subCode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KortexError(null);
    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.subCode !== undefined) {
      resolved.subCode = msg.subCode;
    }
    else {
      resolved.subCode = 0
    }

    return resolved;
    }
};

module.exports = KortexError;
