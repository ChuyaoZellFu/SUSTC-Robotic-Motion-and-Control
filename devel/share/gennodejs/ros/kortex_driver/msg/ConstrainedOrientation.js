// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Orientation = require('./Orientation.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');

//-----------------------------------------------------------

class ConstrainedOrientation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_orientation = null;
      this.constraint = null;
    }
    else {
      if (initObj.hasOwnProperty('target_orientation')) {
        this.target_orientation = initObj.target_orientation
      }
      else {
        this.target_orientation = new Orientation();
      }
      if (initObj.hasOwnProperty('constraint')) {
        this.constraint = initObj.constraint
      }
      else {
        this.constraint = new CartesianTrajectoryConstraint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConstrainedOrientation
    // Serialize message field [target_orientation]
    bufferOffset = Orientation.serialize(obj.target_orientation, buffer, bufferOffset);
    // Serialize message field [constraint]
    bufferOffset = CartesianTrajectoryConstraint.serialize(obj.constraint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConstrainedOrientation
    let len;
    let data = new ConstrainedOrientation(null);
    // Deserialize message field [target_orientation]
    data.target_orientation = Orientation.deserialize(buffer, bufferOffset);
    // Deserialize message field [constraint]
    data.constraint = CartesianTrajectoryConstraint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CartesianTrajectoryConstraint.getMessageSize(object.constraint);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConstrainedOrientation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '15f399eb15570e6ba491b979fdea9cfd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Orientation target_orientation
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Orientation
    
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint
    
    CartesianTrajectoryConstraint_type oneof_type
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint_type
    
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
    const resolved = new ConstrainedOrientation(null);
    if (msg.target_orientation !== undefined) {
      resolved.target_orientation = Orientation.Resolve(msg.target_orientation)
    }
    else {
      resolved.target_orientation = new Orientation()
    }

    if (msg.constraint !== undefined) {
      resolved.constraint = CartesianTrajectoryConstraint.Resolve(msg.constraint)
    }
    else {
      resolved.constraint = new CartesianTrajectoryConstraint()
    }

    return resolved;
    }
};

module.exports = ConstrainedOrientation;
