// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Pose = require('./Pose.js');

//-----------------------------------------------------------

class CartesianWaypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.reference_frame = null;
      this.maximum_linear_velocity = null;
      this.maximum_angular_velocity = null;
      this.blending_radius = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new Pose();
      }
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = 0;
      }
      if (initObj.hasOwnProperty('maximum_linear_velocity')) {
        this.maximum_linear_velocity = initObj.maximum_linear_velocity
      }
      else {
        this.maximum_linear_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('maximum_angular_velocity')) {
        this.maximum_angular_velocity = initObj.maximum_angular_velocity
      }
      else {
        this.maximum_angular_velocity = 0.0;
      }
      if (initObj.hasOwnProperty('blending_radius')) {
        this.blending_radius = initObj.blending_radius
      }
      else {
        this.blending_radius = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianWaypoint
    // Serialize message field [pose]
    bufferOffset = Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.uint32(obj.reference_frame, buffer, bufferOffset);
    // Serialize message field [maximum_linear_velocity]
    bufferOffset = _serializer.float32(obj.maximum_linear_velocity, buffer, bufferOffset);
    // Serialize message field [maximum_angular_velocity]
    bufferOffset = _serializer.float32(obj.maximum_angular_velocity, buffer, bufferOffset);
    // Serialize message field [blending_radius]
    bufferOffset = _serializer.float32(obj.blending_radius, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianWaypoint
    let len;
    let data = new CartesianWaypoint(null);
    // Deserialize message field [pose]
    data.pose = Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [maximum_linear_velocity]
    data.maximum_linear_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [maximum_angular_velocity]
    data.maximum_angular_velocity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [blending_radius]
    data.blending_radius = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianWaypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '057d3bd32497f1e96612efc0fb6ef690';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Pose pose
    uint32 reference_frame
    float32 maximum_linear_velocity
    float32 maximum_angular_velocity
    float32 blending_radius
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianWaypoint(null);
    if (msg.pose !== undefined) {
      resolved.pose = Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new Pose()
    }

    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = 0
    }

    if (msg.maximum_linear_velocity !== undefined) {
      resolved.maximum_linear_velocity = msg.maximum_linear_velocity;
    }
    else {
      resolved.maximum_linear_velocity = 0.0
    }

    if (msg.maximum_angular_velocity !== undefined) {
      resolved.maximum_angular_velocity = msg.maximum_angular_velocity;
    }
    else {
      resolved.maximum_angular_velocity = 0.0
    }

    if (msg.blending_radius !== undefined) {
      resolved.blending_radius = msg.blending_radius;
    }
    else {
      resolved.blending_radius = 0.0
    }

    return resolved;
    }
};

module.exports = CartesianWaypoint;
