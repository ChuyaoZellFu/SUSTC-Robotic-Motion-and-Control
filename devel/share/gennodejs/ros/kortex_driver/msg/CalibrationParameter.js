// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CalibrationParameter_value = require('./CalibrationParameter_value.js');

//-----------------------------------------------------------

class CalibrationParameter {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.calibration_parameter_identifier = null;
      this.oneof_value = null;
    }
    else {
      if (initObj.hasOwnProperty('calibration_parameter_identifier')) {
        this.calibration_parameter_identifier = initObj.calibration_parameter_identifier
      }
      else {
        this.calibration_parameter_identifier = 0;
      }
      if (initObj.hasOwnProperty('oneof_value')) {
        this.oneof_value = initObj.oneof_value
      }
      else {
        this.oneof_value = new CalibrationParameter_value();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationParameter
    // Serialize message field [calibration_parameter_identifier]
    bufferOffset = _serializer.uint32(obj.calibration_parameter_identifier, buffer, bufferOffset);
    // Serialize message field [oneof_value]
    bufferOffset = CalibrationParameter_value.serialize(obj.oneof_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationParameter
    let len;
    let data = new CalibrationParameter(null);
    // Deserialize message field [calibration_parameter_identifier]
    data.calibration_parameter_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [oneof_value]
    data.oneof_value = CalibrationParameter_value.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CalibrationParameter_value.getMessageSize(object.oneof_value);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CalibrationParameter';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9d31a3bff291563e3d32bb702a8bed2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 calibration_parameter_identifier
    CalibrationParameter_value oneof_value
    ================================================================================
    MSG: kortex_driver/CalibrationParameter_value
    
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
    const resolved = new CalibrationParameter(null);
    if (msg.calibration_parameter_identifier !== undefined) {
      resolved.calibration_parameter_identifier = msg.calibration_parameter_identifier;
    }
    else {
      resolved.calibration_parameter_identifier = 0
    }

    if (msg.oneof_value !== undefined) {
      resolved.oneof_value = CalibrationParameter_value.Resolve(msg.oneof_value)
    }
    else {
      resolved.oneof_value = new CalibrationParameter_value()
    }

    return resolved;
    }
};

module.exports = CalibrationParameter;
