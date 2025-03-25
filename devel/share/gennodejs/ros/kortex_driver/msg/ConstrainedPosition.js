// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Base_Position = require('./Base_Position.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');

//-----------------------------------------------------------

class ConstrainedPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_position = null;
      this.constraint = null;
    }
    else {
      if (initObj.hasOwnProperty('target_position')) {
        this.target_position = initObj.target_position
      }
      else {
        this.target_position = new Base_Position();
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
    // Serializes a message object of type ConstrainedPosition
    // Serialize message field [target_position]
    bufferOffset = Base_Position.serialize(obj.target_position, buffer, bufferOffset);
    // Serialize message field [constraint]
    bufferOffset = CartesianTrajectoryConstraint.serialize(obj.constraint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConstrainedPosition
    let len;
    let data = new ConstrainedPosition(null);
    // Deserialize message field [target_position]
    data.target_position = Base_Position.deserialize(buffer, bufferOffset);
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
    return 'kortex_driver/ConstrainedPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4c5833c3b26267ce0142362e034e1ba3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Base_Position target_position
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Base_Position
    
    float32 x
    float32 y
    float32 z
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
    const resolved = new ConstrainedPosition(null);
    if (msg.target_position !== undefined) {
      resolved.target_position = Base_Position.Resolve(msg.target_position)
    }
    else {
      resolved.target_position = new Base_Position()
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

module.exports = ConstrainedPosition;
