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

class Timestamp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sec = null;
      this.usec = null;
    }
    else {
      if (initObj.hasOwnProperty('sec')) {
        this.sec = initObj.sec
      }
      else {
        this.sec = 0;
      }
      if (initObj.hasOwnProperty('usec')) {
        this.usec = initObj.usec
      }
      else {
        this.usec = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Timestamp
    // Serialize message field [sec]
    bufferOffset = _serializer.uint32(obj.sec, buffer, bufferOffset);
    // Serialize message field [usec]
    bufferOffset = _serializer.uint32(obj.usec, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Timestamp
    let len;
    let data = new Timestamp(null);
    // Deserialize message field [sec]
    data.sec = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [usec]
    data.usec = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Timestamp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90da89d9efe7e712be65708e3d776fa4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sec
    uint32 usec
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Timestamp(null);
    if (msg.sec !== undefined) {
      resolved.sec = msg.sec;
    }
    else {
      resolved.sec = 0
    }

    if (msg.usec !== undefined) {
      resolved.usec = msg.usec;
    }
    else {
      resolved.usec = 0
    }

    return resolved;
    }
};

module.exports = Timestamp;
