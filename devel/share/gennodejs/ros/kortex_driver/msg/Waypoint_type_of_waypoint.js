// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let AngularWaypoint = require('./AngularWaypoint.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');

//-----------------------------------------------------------

class Waypoint_type_of_waypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angular_waypoint = null;
      this.cartesian_waypoint = null;
    }
    else {
      if (initObj.hasOwnProperty('angular_waypoint')) {
        this.angular_waypoint = initObj.angular_waypoint
      }
      else {
        this.angular_waypoint = [];
      }
      if (initObj.hasOwnProperty('cartesian_waypoint')) {
        this.cartesian_waypoint = initObj.cartesian_waypoint
      }
      else {
        this.cartesian_waypoint = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Waypoint_type_of_waypoint
    // Serialize message field [angular_waypoint]
    // Serialize the length for message field [angular_waypoint]
    bufferOffset = _serializer.uint32(obj.angular_waypoint.length, buffer, bufferOffset);
    obj.angular_waypoint.forEach((val) => {
      bufferOffset = AngularWaypoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cartesian_waypoint]
    // Serialize the length for message field [cartesian_waypoint]
    bufferOffset = _serializer.uint32(obj.cartesian_waypoint.length, buffer, bufferOffset);
    obj.cartesian_waypoint.forEach((val) => {
      bufferOffset = CartesianWaypoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Waypoint_type_of_waypoint
    let len;
    let data = new Waypoint_type_of_waypoint(null);
    // Deserialize message field [angular_waypoint]
    // Deserialize array length for message field [angular_waypoint]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.angular_waypoint = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.angular_waypoint[i] = AngularWaypoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cartesian_waypoint]
    // Deserialize array length for message field [cartesian_waypoint]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cartesian_waypoint = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cartesian_waypoint[i] = CartesianWaypoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.angular_waypoint.forEach((val) => {
      length += AngularWaypoint.getMessageSize(val);
    });
    length += 40 * object.cartesian_waypoint.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Waypoint_type_of_waypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90682b2ce9c17ef82e25a79e7e035287';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    AngularWaypoint[] angular_waypoint
    CartesianWaypoint[] cartesian_waypoint
    ================================================================================
    MSG: kortex_driver/AngularWaypoint
    
    float32[] angles
    float32[] maximum_velocities
    float32 duration
    ================================================================================
    MSG: kortex_driver/CartesianWaypoint
    
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
    const resolved = new Waypoint_type_of_waypoint(null);
    if (msg.angular_waypoint !== undefined) {
      resolved.angular_waypoint = new Array(msg.angular_waypoint.length);
      for (let i = 0; i < resolved.angular_waypoint.length; ++i) {
        resolved.angular_waypoint[i] = AngularWaypoint.Resolve(msg.angular_waypoint[i]);
      }
    }
    else {
      resolved.angular_waypoint = []
    }

    if (msg.cartesian_waypoint !== undefined) {
      resolved.cartesian_waypoint = new Array(msg.cartesian_waypoint.length);
      for (let i = 0; i < resolved.cartesian_waypoint.length; ++i) {
        resolved.cartesian_waypoint[i] = CartesianWaypoint.Resolve(msg.cartesian_waypoint[i]);
      }
    }
    else {
      resolved.cartesian_waypoint = []
    }

    return resolved;
    }
};

module.exports = Waypoint_type_of_waypoint;
