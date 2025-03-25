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

class KinematicTrajectoryConstraints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angular_velocities = null;
      this.linear_velocity = null;
      this.angular_velocity = null;
    }
    else {
      if (initObj.hasOwnProperty('angular_velocities')) {
        this.angular_velocities = initObj.angular_velocities
      }
      else {
        this.angular_velocities = [];
      }
      if (initObj.hasOwnProperty('linear_velocity')) {
        this.linear_velocity = initObj.linear_velocity
      }
      else {
        this.linear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KinematicTrajectoryConstraints
    // Serialize message field [angular_velocities]
    bufferOffset = _arraySerializer.float32(obj.angular_velocities, buffer, bufferOffset, null);
    // Serialize message field [linear_velocity]
    bufferOffset = _serializer.float32(obj.linear_velocity, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = _serializer.float32(obj.angular_velocity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KinematicTrajectoryConstraints
    let len;
    let data = new KinematicTrajectoryConstraints(null);
    // Deserialize message field [angular_velocities]
    data.angular_velocities = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [linear_velocity]
    data.linear_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.angular_velocities.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/KinematicTrajectoryConstraints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f0c906b29207ad54df6adc023e55732';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32[] angular_velocities
    float32 linear_velocity
    float32 angular_velocity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KinematicTrajectoryConstraints(null);
    if (msg.angular_velocities !== undefined) {
      resolved.angular_velocities = msg.angular_velocities;
    }
    else {
      resolved.angular_velocities = []
    }

    if (msg.linear_velocity !== undefined) {
      resolved.linear_velocity = msg.linear_velocity;
    }
    else {
      resolved.linear_velocity = 0.0
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = msg.angular_velocity;
    }
    else {
      resolved.angular_velocity = 0.0
    }

    return resolved;
    }
};

module.exports = KinematicTrajectoryConstraints;
