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

class VectorDriveParameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kpq = null;
      this.kiq = null;
      this.kpd = null;
      this.kid = null;
    }
    else {
      if (initObj.hasOwnProperty('kpq')) {
        this.kpq = initObj.kpq
      }
      else {
        this.kpq = 0.0;
      }
      if (initObj.hasOwnProperty('kiq')) {
        this.kiq = initObj.kiq
      }
      else {
        this.kiq = 0.0;
      }
      if (initObj.hasOwnProperty('kpd')) {
        this.kpd = initObj.kpd
      }
      else {
        this.kpd = 0.0;
      }
      if (initObj.hasOwnProperty('kid')) {
        this.kid = initObj.kid
      }
      else {
        this.kid = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VectorDriveParameters
    // Serialize message field [kpq]
    bufferOffset = _serializer.float32(obj.kpq, buffer, bufferOffset);
    // Serialize message field [kiq]
    bufferOffset = _serializer.float32(obj.kiq, buffer, bufferOffset);
    // Serialize message field [kpd]
    bufferOffset = _serializer.float32(obj.kpd, buffer, bufferOffset);
    // Serialize message field [kid]
    bufferOffset = _serializer.float32(obj.kid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VectorDriveParameters
    let len;
    let data = new VectorDriveParameters(null);
    // Deserialize message field [kpq]
    data.kpq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kiq]
    data.kiq = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kpd]
    data.kpd = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [kid]
    data.kid = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/VectorDriveParameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '30e295016ac63da10cbe59e969bb6401';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    float32 kpq
    float32 kiq
    float32 kpd
    float32 kid
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VectorDriveParameters(null);
    if (msg.kpq !== undefined) {
      resolved.kpq = msg.kpq;
    }
    else {
      resolved.kpq = 0.0
    }

    if (msg.kiq !== undefined) {
      resolved.kiq = msg.kiq;
    }
    else {
      resolved.kiq = 0.0
    }

    if (msg.kpd !== undefined) {
      resolved.kpd = msg.kpd;
    }
    else {
      resolved.kpd = 0.0
    }

    if (msg.kid !== undefined) {
      resolved.kid = msg.kid;
    }
    else {
      resolved.kid = 0.0
    }

    return resolved;
    }
};

module.exports = VectorDriveParameters;
