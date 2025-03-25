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

class ApiOptions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timeout_ms = null;
    }
    else {
      if (initObj.hasOwnProperty('timeout_ms')) {
        this.timeout_ms = initObj.timeout_ms
      }
      else {
        this.timeout_ms = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ApiOptions
    // Serialize message field [timeout_ms]
    bufferOffset = _serializer.uint32(obj.timeout_ms, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ApiOptions
    let len;
    let data = new ApiOptions(null);
    // Deserialize message field [timeout_ms]
    data.timeout_ms = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ApiOptions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88b8e888a6c1e4d612523746429635b7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 timeout_ms
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ApiOptions(null);
    if (msg.timeout_ms !== undefined) {
      resolved.timeout_ms = msg.timeout_ms;
    }
    else {
      resolved.timeout_ms = 0
    }

    return resolved;
    }
};

module.exports = ApiOptions;
