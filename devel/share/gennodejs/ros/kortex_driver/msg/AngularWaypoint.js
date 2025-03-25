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

class AngularWaypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angles = null;
      this.maximum_velocities = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('angles')) {
        this.angles = initObj.angles
      }
      else {
        this.angles = [];
      }
      if (initObj.hasOwnProperty('maximum_velocities')) {
        this.maximum_velocities = initObj.maximum_velocities
      }
      else {
        this.maximum_velocities = [];
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
    // Serializes a message object of type AngularWaypoint
    // Serialize message field [angles]
    bufferOffset = _arraySerializer.float32(obj.angles, buffer, bufferOffset, null);
    // Serialize message field [maximum_velocities]
    bufferOffset = _arraySerializer.float32(obj.maximum_velocities, buffer, bufferOffset, null);
    // Serialize message field [duration]
    bufferOffset = _serializer.float32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AngularWaypoint
    let len;
    let data = new AngularWaypoint(null);
    // Deserialize message field [angles]
    data.angles = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [maximum_velocities]
    data.maximum_velocities = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [duration]
    data.duration = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.angles.length;
    length += 4 * object.maximum_velocities.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/AngularWaypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2389f0b849f9b353e58e97764b9bfc7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32[] angles
    float32[] maximum_velocities
    float32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AngularWaypoint(null);
    if (msg.angles !== undefined) {
      resolved.angles = msg.angles;
    }
    else {
      resolved.angles = []
    }

    if (msg.maximum_velocities !== undefined) {
      resolved.maximum_velocities = msg.maximum_velocities;
    }
    else {
      resolved.maximum_velocities = []
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

module.exports = AngularWaypoint;
