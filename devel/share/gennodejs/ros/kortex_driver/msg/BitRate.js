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

class BitRate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BitRate
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BitRate
    let len;
    let data = new BitRate(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BitRate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '472a4e4218d43df9cfed86c61a2322e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 BITRATE_UNSPECIFIED = 0
    
    uint32 BITRATE_10_MBPS = 1
    
    uint32 BITRATE_15_MBPS = 2
    
    uint32 BITRATE_20_MBPS = 3
    
    uint32 BITRATE_25_MBPS = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BitRate(null);
    return resolved;
    }
};

// Constants for message
BitRate.Constants = {
  BITRATE_UNSPECIFIED: 0,
  BITRATE_10_MBPS: 1,
  BITRATE_15_MBPS: 2,
  BITRATE_20_MBPS: 3,
  BITRATE_25_MBPS: 4,
}

module.exports = BitRate;
