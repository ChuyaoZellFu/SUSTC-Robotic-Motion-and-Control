// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Twist = require('./Twist.js');

//-----------------------------------------------------------

class TwistCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reference_frame = null;
      this.twist = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = 0;
      }
      if (initObj.hasOwnProperty('twist')) {
        this.twist = initObj.twist
      }
      else {
        this.twist = new Twist();
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TwistCommand
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.uint32(obj.reference_frame, buffer, bufferOffset);
    // Serialize message field [twist]
    bufferOffset = Twist.serialize(obj.twist, buffer, bufferOffset);
    // Serialize message field [duration]
    bufferOffset = _serializer.uint32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TwistCommand
    let len;
    let data = new TwistCommand(null);
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [twist]
    data.twist = Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [duration]
    data.duration = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TwistCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16dcfd20a022a10eea1f05e5a9cbb18a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 reference_frame
    Twist twist
    uint32 duration
    ================================================================================
    MSG: kortex_driver/Twist
    
    float32 linear_x
    float32 linear_y
    float32 linear_z
    float32 angular_x
    float32 angular_y
    float32 angular_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TwistCommand(null);
    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = 0
    }

    if (msg.twist !== undefined) {
      resolved.twist = Twist.Resolve(msg.twist)
    }
    else {
      resolved.twist = new Twist()
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
    }

    return resolved;
    }
};

module.exports = TwistCommand;
