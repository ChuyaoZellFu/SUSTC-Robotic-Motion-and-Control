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

class Twist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.linear_x = null;
      this.linear_y = null;
      this.linear_z = null;
      this.angular_x = null;
      this.angular_y = null;
      this.angular_z = null;
    }
    else {
      if (initObj.hasOwnProperty('linear_x')) {
        this.linear_x = initObj.linear_x
      }
      else {
        this.linear_x = 0.0;
      }
      if (initObj.hasOwnProperty('linear_y')) {
        this.linear_y = initObj.linear_y
      }
      else {
        this.linear_y = 0.0;
      }
      if (initObj.hasOwnProperty('linear_z')) {
        this.linear_z = initObj.linear_z
      }
      else {
        this.linear_z = 0.0;
      }
      if (initObj.hasOwnProperty('angular_x')) {
        this.angular_x = initObj.angular_x
      }
      else {
        this.angular_x = 0.0;
      }
      if (initObj.hasOwnProperty('angular_y')) {
        this.angular_y = initObj.angular_y
      }
      else {
        this.angular_y = 0.0;
      }
      if (initObj.hasOwnProperty('angular_z')) {
        this.angular_z = initObj.angular_z
      }
      else {
        this.angular_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Twist
    // Serialize message field [linear_x]
    bufferOffset = _serializer.float32(obj.linear_x, buffer, bufferOffset);
    // Serialize message field [linear_y]
    bufferOffset = _serializer.float32(obj.linear_y, buffer, bufferOffset);
    // Serialize message field [linear_z]
    bufferOffset = _serializer.float32(obj.linear_z, buffer, bufferOffset);
    // Serialize message field [angular_x]
    bufferOffset = _serializer.float32(obj.angular_x, buffer, bufferOffset);
    // Serialize message field [angular_y]
    bufferOffset = _serializer.float32(obj.angular_y, buffer, bufferOffset);
    // Serialize message field [angular_z]
    bufferOffset = _serializer.float32(obj.angular_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Twist
    let len;
    let data = new Twist(null);
    // Deserialize message field [linear_x]
    data.linear_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_y]
    data.linear_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_z]
    data.linear_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_x]
    data.angular_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_y]
    data.angular_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_z]
    data.angular_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Twist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aadf583bd82c7a5a42adeb6e230fba63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 linear_x
    float32 linear_y
    float32 linear_z
    float32 angular_x
    float32 angular_y
    float32 angular_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Twist(null);
    if (msg.linear_x !== undefined) {
      resolved.linear_x = msg.linear_x;
    }
    else {
      resolved.linear_x = 0.0
    }

    if (msg.linear_y !== undefined) {
      resolved.linear_y = msg.linear_y;
    }
    else {
      resolved.linear_y = 0.0
    }

    if (msg.linear_z !== undefined) {
      resolved.linear_z = msg.linear_z;
    }
    else {
      resolved.linear_z = 0.0
    }

    if (msg.angular_x !== undefined) {
      resolved.angular_x = msg.angular_x;
    }
    else {
      resolved.angular_x = 0.0
    }

    if (msg.angular_y !== undefined) {
      resolved.angular_y = msg.angular_y;
    }
    else {
      resolved.angular_y = 0.0
    }

    if (msg.angular_z !== undefined) {
      resolved.angular_z = msg.angular_z;
    }
    else {
      resolved.angular_z = 0.0
    }

    return resolved;
    }
};

module.exports = Twist;
