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

class DesiredSpeeds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.linear = null;
      this.angular = null;
      this.joint_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('linear')) {
        this.linear = initObj.linear
      }
      else {
        this.linear = 0.0;
      }
      if (initObj.hasOwnProperty('angular')) {
        this.angular = initObj.angular
      }
      else {
        this.angular = 0.0;
      }
      if (initObj.hasOwnProperty('joint_speed')) {
        this.joint_speed = initObj.joint_speed
      }
      else {
        this.joint_speed = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DesiredSpeeds
    // Serialize message field [linear]
    bufferOffset = _serializer.float32(obj.linear, buffer, bufferOffset);
    // Serialize message field [angular]
    bufferOffset = _serializer.float32(obj.angular, buffer, bufferOffset);
    // Serialize message field [joint_speed]
    bufferOffset = _arraySerializer.float32(obj.joint_speed, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DesiredSpeeds
    let len;
    let data = new DesiredSpeeds(null);
    // Deserialize message field [linear]
    data.linear = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular]
    data.angular = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint_speed]
    data.joint_speed = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_speed.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DesiredSpeeds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f61e0fe165a7f46b2508fd0832ff820';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 linear
    float32 angular
    float32[] joint_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DesiredSpeeds(null);
    if (msg.linear !== undefined) {
      resolved.linear = msg.linear;
    }
    else {
      resolved.linear = 0.0
    }

    if (msg.angular !== undefined) {
      resolved.angular = msg.angular;
    }
    else {
      resolved.angular = 0.0
    }

    if (msg.joint_speed !== undefined) {
      resolved.joint_speed = msg.joint_speed;
    }
    else {
      resolved.joint_speed = []
    }

    return resolved;
    }
};

module.exports = DesiredSpeeds;
