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

class CalibrationParameter_value {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.signedIntValue = null;
      this.unsignedIntValue = null;
      this.floatValue = null;
    }
    else {
      if (initObj.hasOwnProperty('signedIntValue')) {
        this.signedIntValue = initObj.signedIntValue
      }
      else {
        this.signedIntValue = [];
      }
      if (initObj.hasOwnProperty('unsignedIntValue')) {
        this.unsignedIntValue = initObj.unsignedIntValue
      }
      else {
        this.unsignedIntValue = [];
      }
      if (initObj.hasOwnProperty('floatValue')) {
        this.floatValue = initObj.floatValue
      }
      else {
        this.floatValue = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationParameter_value
    // Serialize message field [signedIntValue]
    bufferOffset = _arraySerializer.uint32(obj.signedIntValue, buffer, bufferOffset, null);
    // Serialize message field [unsignedIntValue]
    bufferOffset = _arraySerializer.uint32(obj.unsignedIntValue, buffer, bufferOffset, null);
    // Serialize message field [floatValue]
    bufferOffset = _arraySerializer.uint32(obj.floatValue, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationParameter_value
    let len;
    let data = new CalibrationParameter_value(null);
    // Deserialize message field [signedIntValue]
    data.signedIntValue = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [unsignedIntValue]
    data.unsignedIntValue = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [floatValue]
    data.floatValue = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.signedIntValue.length;
    length += 4 * object.unsignedIntValue.length;
    length += 4 * object.floatValue.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CalibrationParameter_value';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2b45c1a772c79ef326799a208a383734';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32[] signedIntValue
    uint32[] unsignedIntValue
    uint32[] floatValue
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationParameter_value(null);
    if (msg.signedIntValue !== undefined) {
      resolved.signedIntValue = msg.signedIntValue;
    }
    else {
      resolved.signedIntValue = []
    }

    if (msg.unsignedIntValue !== undefined) {
      resolved.unsignedIntValue = msg.unsignedIntValue;
    }
    else {
      resolved.unsignedIntValue = []
    }

    if (msg.floatValue !== undefined) {
      resolved.floatValue = msg.floatValue;
    }
    else {
      resolved.floatValue = []
    }

    return resolved;
    }
};

module.exports = CalibrationParameter_value;
