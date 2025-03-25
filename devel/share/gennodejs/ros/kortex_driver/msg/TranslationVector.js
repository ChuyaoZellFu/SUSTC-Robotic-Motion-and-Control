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

class TranslationVector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t_x = null;
      this.t_y = null;
      this.t_z = null;
    }
    else {
      if (initObj.hasOwnProperty('t_x')) {
        this.t_x = initObj.t_x
      }
      else {
        this.t_x = 0.0;
      }
      if (initObj.hasOwnProperty('t_y')) {
        this.t_y = initObj.t_y
      }
      else {
        this.t_y = 0.0;
      }
      if (initObj.hasOwnProperty('t_z')) {
        this.t_z = initObj.t_z
      }
      else {
        this.t_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TranslationVector
    // Serialize message field [t_x]
    bufferOffset = _serializer.float32(obj.t_x, buffer, bufferOffset);
    // Serialize message field [t_y]
    bufferOffset = _serializer.float32(obj.t_y, buffer, bufferOffset);
    // Serialize message field [t_z]
    bufferOffset = _serializer.float32(obj.t_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TranslationVector
    let len;
    let data = new TranslationVector(null);
    // Deserialize message field [t_x]
    data.t_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [t_y]
    data.t_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [t_z]
    data.t_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TranslationVector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1c59f12cff6754b509028be4dd466647';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 t_x
    float32 t_y
    float32 t_z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TranslationVector(null);
    if (msg.t_x !== undefined) {
      resolved.t_x = msg.t_x;
    }
    else {
      resolved.t_x = 0.0
    }

    if (msg.t_y !== undefined) {
      resolved.t_y = msg.t_y;
    }
    else {
      resolved.t_y = 0.0
    }

    if (msg.t_z !== undefined) {
      resolved.t_z = msg.t_z;
    }
    else {
      resolved.t_z = 0.0
    }

    return resolved;
    }
};

module.exports = TranslationVector;
