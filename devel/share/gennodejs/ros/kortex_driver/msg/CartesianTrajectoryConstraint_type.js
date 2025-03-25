// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianSpeed = require('./CartesianSpeed.js');

//-----------------------------------------------------------

class CartesianTrajectoryConstraint_type {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = [];
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianTrajectoryConstraint_type
    // Serialize message field [speed]
    // Serialize the length for message field [speed]
    bufferOffset = _serializer.uint32(obj.speed.length, buffer, bufferOffset);
    obj.speed.forEach((val) => {
      bufferOffset = CartesianSpeed.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [duration]
    bufferOffset = _arraySerializer.uint32(obj.duration, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianTrajectoryConstraint_type
    let len;
    let data = new CartesianTrajectoryConstraint_type(null);
    // Deserialize message field [speed]
    // Deserialize array length for message field [speed]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.speed = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.speed[i] = CartesianSpeed.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [duration]
    data.duration = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.speed.length;
    length += 4 * object.duration.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianTrajectoryConstraint_type';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e14536e767e1f700827f285e4e6fd832';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    CartesianSpeed[] speed
    uint32[] duration
    ================================================================================
    MSG: kortex_driver/CartesianSpeed
    
    float32 translation
    float32 orientation
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianTrajectoryConstraint_type(null);
    if (msg.speed !== undefined) {
      resolved.speed = new Array(msg.speed.length);
      for (let i = 0; i < resolved.speed.length; ++i) {
        resolved.speed[i] = CartesianSpeed.Resolve(msg.speed[i]);
      }
    }
    else {
      resolved.speed = []
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = []
    }

    return resolved;
    }
};

module.exports = CartesianTrajectoryConstraint_type;
