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

class SensorSettings {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
      this.resolution = null;
      this.frame_rate = null;
      this.bit_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = 0;
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0;
      }
      if (initObj.hasOwnProperty('frame_rate')) {
        this.frame_rate = initObj.frame_rate
      }
      else {
        this.frame_rate = 0;
      }
      if (initObj.hasOwnProperty('bit_rate')) {
        this.bit_rate = initObj.bit_rate
      }
      else {
        this.bit_rate = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorSettings
    // Serialize message field [sensor]
    bufferOffset = _serializer.uint32(obj.sensor, buffer, bufferOffset);
    // Serialize message field [resolution]
    bufferOffset = _serializer.uint32(obj.resolution, buffer, bufferOffset);
    // Serialize message field [frame_rate]
    bufferOffset = _serializer.uint32(obj.frame_rate, buffer, bufferOffset);
    // Serialize message field [bit_rate]
    bufferOffset = _serializer.uint32(obj.bit_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorSettings
    let len;
    let data = new SensorSettings(null);
    // Deserialize message field [sensor]
    data.sensor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [resolution]
    data.resolution = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [frame_rate]
    data.frame_rate = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bit_rate]
    data.bit_rate = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SensorSettings';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1b1f6cd3d6eb9aa0d6912b66a0f489bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sensor
    uint32 resolution
    uint32 frame_rate
    uint32 bit_rate
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorSettings(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = msg.sensor;
    }
    else {
      resolved.sensor = 0
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0
    }

    if (msg.frame_rate !== undefined) {
      resolved.frame_rate = msg.frame_rate;
    }
    else {
      resolved.frame_rate = 0
    }

    if (msg.bit_rate !== undefined) {
      resolved.bit_rate = msg.bit_rate;
    }
    else {
      resolved.bit_rate = 0
    }

    return resolved;
    }
};

module.exports = SensorSettings;
