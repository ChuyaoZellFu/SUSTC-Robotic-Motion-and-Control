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

class JointLimitation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_identifier = null;
      this.type = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_identifier')) {
        this.joint_identifier = initObj.joint_identifier
      }
      else {
        this.joint_identifier = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointLimitation
    // Serialize message field [joint_identifier]
    bufferOffset = _serializer.uint32(obj.joint_identifier, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint32(obj.type, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointLimitation
    let len;
    let data = new JointLimitation(null);
    // Deserialize message field [joint_identifier]
    data.joint_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointLimitation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c7e8a86926cdf71a09bb6314d52e9586';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 joint_identifier
    uint32 type
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointLimitation(null);
    if (msg.joint_identifier !== undefined) {
      resolved.joint_identifier = msg.joint_identifier;
    }
    else {
      resolved.joint_identifier = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = JointLimitation;
