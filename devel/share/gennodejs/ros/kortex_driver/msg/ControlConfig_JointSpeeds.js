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

class ControlConfig_JointSpeeds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_speed')) {
        this.joint_speed = initObj.joint_speed
      }
      else {
        this.joint_speed = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlConfig_JointSpeeds
    // Serialize message field [joint_speed]
    bufferOffset = _arraySerializer.float32(obj.joint_speed, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlConfig_JointSpeeds
    let len;
    let data = new ControlConfig_JointSpeeds(null);
    // Deserialize message field [joint_speed]
    data.joint_speed = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.joint_speed.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControlConfig_JointSpeeds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33fe593b138a7242a779a630fd20122e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32[] joint_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlConfig_JointSpeeds(null);
    if (msg.joint_speed !== undefined) {
      resolved.joint_speed = msg.joint_speed;
    }
    else {
      resolved.joint_speed = []
    }

    return resolved;
    }
};

module.exports = ControlConfig_JointSpeeds;
