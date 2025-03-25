// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');

//-----------------------------------------------------------

class Waypoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.oneof_type_of_waypoint = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('oneof_type_of_waypoint')) {
        this.oneof_type_of_waypoint = initObj.oneof_type_of_waypoint
      }
      else {
        this.oneof_type_of_waypoint = new Waypoint_type_of_waypoint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Waypoint
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [oneof_type_of_waypoint]
    bufferOffset = Waypoint_type_of_waypoint.serialize(obj.oneof_type_of_waypoint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Waypoint
    let len;
    let data = new Waypoint(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [oneof_type_of_waypoint]
    data.oneof_type_of_waypoint = Waypoint_type_of_waypoint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += Waypoint_type_of_waypoint.getMessageSize(object.oneof_type_of_waypoint);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Waypoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '936edf0520133d6221befe691467b5ce';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string name
    Waypoint_type_of_waypoint oneof_type_of_waypoint
    ================================================================================
    MSG: kortex_driver/Waypoint_type_of_waypoint
    
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
    const resolved = new Waypoint(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.oneof_type_of_waypoint !== undefined) {
      resolved.oneof_type_of_waypoint = Waypoint_type_of_waypoint.Resolve(msg.oneof_type_of_waypoint)
    }
    else {
      resolved.oneof_type_of_waypoint = new Waypoint_type_of_waypoint()
    }

    return resolved;
    }
};

module.exports = Waypoint;
