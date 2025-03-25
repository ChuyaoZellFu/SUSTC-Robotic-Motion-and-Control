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

class NotificationOptions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
      this.rate_m_sec = null;
      this.threshold_value = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('rate_m_sec')) {
        this.rate_m_sec = initObj.rate_m_sec
      }
      else {
        this.rate_m_sec = 0;
      }
      if (initObj.hasOwnProperty('threshold_value')) {
        this.threshold_value = initObj.threshold_value
      }
      else {
        this.threshold_value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NotificationOptions
    // Serialize message field [type]
    bufferOffset = _serializer.uint32(obj.type, buffer, bufferOffset);
    // Serialize message field [rate_m_sec]
    bufferOffset = _serializer.uint32(obj.rate_m_sec, buffer, bufferOffset);
    // Serialize message field [threshold_value]
    bufferOffset = _serializer.float32(obj.threshold_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NotificationOptions
    let len;
    let data = new NotificationOptions(null);
    // Deserialize message field [type]
    data.type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [rate_m_sec]
    data.rate_m_sec = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [threshold_value]
    data.threshold_value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/NotificationOptions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '756dde28efcdf67e7515ead30453e781';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 type
    uint32 rate_m_sec
    float32 threshold_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NotificationOptions(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.rate_m_sec !== undefined) {
      resolved.rate_m_sec = msg.rate_m_sec;
    }
    else {
      resolved.rate_m_sec = 0
    }

    if (msg.threshold_value !== undefined) {
      resolved.threshold_value = msg.threshold_value;
    }
    else {
      resolved.threshold_value = 0.0
    }

    return resolved;
    }
};

module.exports = NotificationOptions;
