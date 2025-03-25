// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');

//-----------------------------------------------------------

class CartesianTrajectoryConstraint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.oneof_type = null;
    }
    else {
      if (initObj.hasOwnProperty('oneof_type')) {
        this.oneof_type = initObj.oneof_type
      }
      else {
        this.oneof_type = new CartesianTrajectoryConstraint_type();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianTrajectoryConstraint
    // Serialize message field [oneof_type]
    bufferOffset = CartesianTrajectoryConstraint_type.serialize(obj.oneof_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianTrajectoryConstraint
    let len;
    let data = new CartesianTrajectoryConstraint(null);
    // Deserialize message field [oneof_type]
    data.oneof_type = CartesianTrajectoryConstraint_type.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CartesianTrajectoryConstraint_type.getMessageSize(object.oneof_type);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CartesianTrajectoryConstraint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4758c371e633c2206a71f0c6833f6d5a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new CartesianTrajectoryConstraint(null);
    if (msg.oneof_type !== undefined) {
      resolved.oneof_type = CartesianTrajectoryConstraint_type.Resolve(msg.oneof_type)
    }
    else {
      resolved.oneof_type = new CartesianTrajectoryConstraint_type()
    }

    return resolved;
    }
};

module.exports = CartesianTrajectoryConstraint;
