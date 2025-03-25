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

class StepResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loop_selection = null;
      this.amplitude = null;
      this.step_delay = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('loop_selection')) {
        this.loop_selection = initObj.loop_selection
      }
      else {
        this.loop_selection = 0;
      }
      if (initObj.hasOwnProperty('amplitude')) {
        this.amplitude = initObj.amplitude
      }
      else {
        this.amplitude = 0.0;
      }
      if (initObj.hasOwnProperty('step_delay')) {
        this.step_delay = initObj.step_delay
      }
      else {
        this.step_delay = 0.0;
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
    // Serializes a message object of type StepResponse
    // Serialize message field [loop_selection]
    bufferOffset = _serializer.uint32(obj.loop_selection, buffer, bufferOffset);
    // Serialize message field [amplitude]
    bufferOffset = _serializer.float32(obj.amplitude, buffer, bufferOffset);
    // Serialize message field [step_delay]
    bufferOffset = _serializer.float32(obj.step_delay, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StepResponse
    let len;
    let data = new StepResponse(null);
    // Deserialize message field [loop_selection]
    data.loop_selection = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [amplitude]
    data.amplitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [step_delay]
    data.step_delay = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/StepResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9ae6749b10a5d88fae7f1a1e8639d93';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 loop_selection
    float32 amplitude
    float32 step_delay
    float32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StepResponse(null);
    if (msg.loop_selection !== undefined) {
      resolved.loop_selection = msg.loop_selection;
    }
    else {
      resolved.loop_selection = 0
    }

    if (msg.amplitude !== undefined) {
      resolved.amplitude = msg.amplitude;
    }
    else {
      resolved.amplitude = 0.0
    }

    if (msg.step_delay !== undefined) {
      resolved.step_delay = msg.step_delay;
    }
    else {
      resolved.step_delay = 0.0
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

module.exports = StepResponse;
