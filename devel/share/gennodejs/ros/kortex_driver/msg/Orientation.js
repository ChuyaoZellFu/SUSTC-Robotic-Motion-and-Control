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

class Orientation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_x = null;
      this.theta_y = null;
      this.theta_z = null;
    }
    else {
      if (initObj.hasOwnProperty('theta_x')) {
        this.theta_x = initObj.theta_x
      }
      else {
        this.theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('theta_y')) {
        this.theta_y = initObj.theta_y
      }
      else {
        this.theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('theta_z')) {
        this.theta_z = initObj.theta_z
      }
      else {
        this.theta_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Orientation
    // Serialize message field [theta_x]
    bufferOffset = _serializer.float32(obj.theta_x, buffer, bufferOffset);
    // Serialize message field [theta_y]
    bufferOffset = _serializer.float32(obj.theta_y, buffer, bufferOffset);
    // Serialize message field [theta_z]
    bufferOffset = _serializer.float32(obj.theta_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Orientation
    let len;
    let data = new Orientation(null);
    // Deserialize message field [theta_x]
    data.theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_y]
    data.theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_z]
    data.theta_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Orientation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2fb1d2b19db735698f3a1ad8c8e9d45b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new Orientation(null);
    if (msg.theta_x !== undefined) {
      resolved.theta_x = msg.theta_x;
    }
    else {
      resolved.theta_x = 0.0
    }

    if (msg.theta_y !== undefined) {
      resolved.theta_y = msg.theta_y;
    }
    else {
      resolved.theta_y = 0.0
    }

    if (msg.theta_z !== undefined) {
      resolved.theta_z = msg.theta_z;
    }
    else {
      resolved.theta_z = 0.0
    }

    return resolved;
    }
};

module.exports = Orientation;
