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

class RampResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_selection = null;
      this.slope = null;
      this.ramp_delay = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_selection')) {
        this.loop_selection = initObj.loop_selection
      }
      else {
        this.loop_selection = 0;
      }
      if (initObj.hasOwnProperty('slope')) {
        this.slope = initObj.slope
      }
      else {
        this.slope = 0.0;
      }
      if (initObj.hasOwnProperty('ramp_delay')) {
        this.ramp_delay = initObj.ramp_delay
      }
      else {
        this.ramp_delay = 0.0;
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RampResponse
    // Serialize message field [loop_selection]
    bufferOffset = _serializer.uint32(obj.loop_selection, buffer, bufferOffset);
    // Serialize message field [slope]
    bufferOffset = _serializer.float32(obj.slope, buffer, bufferOffset);
    // Serialize message field [ramp_delay]
    bufferOffset = _serializer.float32(obj.ramp_delay, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RampResponse
    let len;
    let data = new RampResponse(null);
    // Deserialize message field [loop_selection]
    data.loop_selection = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [slope]
    data.slope = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ramp_delay]
    data.ramp_delay = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RampResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e3f4edf9087b42bf3e2e7ff4ca2b2f9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 loop_selection
    float32 slope
    float32 ramp_delay
    float32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RampResponse(null);
    if (msg.loop_selection !== undefined) {
      resolved.loop_selection = msg.loop_selection;
    }
    else {
      resolved.loop_selection = 0
    }

    if (msg.slope !== undefined) {
      resolved.slope = msg.slope;
    }
    else {
      resolved.slope = 0.0
    }

    if (msg.ramp_delay !== undefined) {
      resolved.ramp_delay = msg.ramp_delay;
    }
    else {
      resolved.ramp_delay = 0.0
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0.0
    }

    return resolved;
    }
};

module.exports = RampResponse;
