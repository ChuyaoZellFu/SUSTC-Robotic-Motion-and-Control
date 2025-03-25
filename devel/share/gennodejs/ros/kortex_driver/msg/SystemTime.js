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

class SystemTime {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sec = null;
      this.min = null;
      this.hour = null;
      this.mday = null;
      this.mon = null;
      this.year = null;
    }
    else {
      if (initObj.hasOwnProperty('sec')) {
        this.sec = initObj.sec
      }
      else {
        this.sec = 0;
      }
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = 0;
      }
      if (initObj.hasOwnProperty('hour')) {
        this.hour = initObj.hour
      }
      else {
        this.hour = 0;
      }
      if (initObj.hasOwnProperty('mday')) {
        this.mday = initObj.mday
      }
      else {
        this.mday = 0;
      }
      if (initObj.hasOwnProperty('mon')) {
        this.mon = initObj.mon
      }
      else {
        this.mon = 0;
      }
      if (initObj.hasOwnProperty('year')) {
        this.year = initObj.year
      }
      else {
        this.year = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SystemTime
    // Serialize message field [sec]
    bufferOffset = _serializer.uint32(obj.sec, buffer, bufferOffset);
    // Serialize message field [min]
    bufferOffset = _serializer.uint32(obj.min, buffer, bufferOffset);
    // Serialize message field [hour]
    bufferOffset = _serializer.uint32(obj.hour, buffer, bufferOffset);
    // Serialize message field [mday]
    bufferOffset = _serializer.uint32(obj.mday, buffer, bufferOffset);
    // Serialize message field [mon]
    bufferOffset = _serializer.uint32(obj.mon, buffer, bufferOffset);
    // Serialize message field [year]
    bufferOffset = _serializer.uint32(obj.year, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SystemTime
    let len;
    let data = new SystemTime(null);
    // Deserialize message field [sec]
    data.sec = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [min]
    data.min = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [hour]
    data.hour = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mday]
    data.mday = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mon]
    data.mon = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [year]
    data.year = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SystemTime';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c3c3b4e6f22b647897641173820f05db';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sec
    uint32 min
    uint32 hour
    uint32 mday
    uint32 mon
    uint32 year
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SystemTime(null);
    if (msg.sec !== undefined) {
      resolved.sec = msg.sec;
    }
    else {
      resolved.sec = 0
    }

    if (msg.min !== undefined) {
      resolved.min = msg.min;
    }
    else {
      resolved.min = 0
    }

    if (msg.hour !== undefined) {
      resolved.hour = msg.hour;
    }
    else {
      resolved.hour = 0
    }

    if (msg.mday !== undefined) {
      resolved.mday = msg.mday;
    }
    else {
      resolved.mday = 0
    }

    if (msg.mon !== undefined) {
      resolved.mon = msg.mon;
    }
    else {
      resolved.mon = 0
    }

    if (msg.year !== undefined) {
      resolved.year = msg.year;
    }
    else {
      resolved.year = 0
    }

    return resolved;
    }
};

module.exports = SystemTime;
