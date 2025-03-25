// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CalibrationParameter = require('./CalibrationParameter.js');

//-----------------------------------------------------------

class Calibration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.calibration_item = null;
      this.calibration_parameter = null;
    }
    else {
      if (initObj.hasOwnProperty('calibration_item')) {
        this.calibration_item = initObj.calibration_item
      }
      else {
        this.calibration_item = 0;
      }
      if (initObj.hasOwnProperty('calibration_parameter')) {
        this.calibration_parameter = initObj.calibration_parameter
      }
      else {
        this.calibration_parameter = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Calibration
    // Serialize message field [calibration_item]
    bufferOffset = _serializer.uint32(obj.calibration_item, buffer, bufferOffset);
    // Serialize message field [calibration_parameter]
    // Serialize the length for message field [calibration_parameter]
    bufferOffset = _serializer.uint32(obj.calibration_parameter.length, buffer, bufferOffset);
    obj.calibration_parameter.forEach((val) => {
      bufferOffset = CalibrationParameter.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Calibration
    let len;
    let data = new Calibration(null);
    // Deserialize message field [calibration_item]
    data.calibration_item = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [calibration_parameter]
    // Deserialize array length for message field [calibration_parameter]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.calibration_parameter = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.calibration_parameter[i] = CalibrationParameter.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.calibration_parameter.forEach((val) => {
      length += CalibrationParameter.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Calibration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e1f62c964640a6b04e3e0de4bfef9f2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 calibration_item
    CalibrationParameter[] calibration_parameter
    ================================================================================
    MSG: kortex_driver/CalibrationParameter
    
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
    const resolved = new Calibration(null);
    if (msg.calibration_item !== undefined) {
      resolved.calibration_item = msg.calibration_item;
    }
    else {
      resolved.calibration_item = 0
    }

    if (msg.calibration_parameter !== undefined) {
      resolved.calibration_parameter = new Array(msg.calibration_parameter.length);
      for (let i = 0; i < resolved.calibration_parameter.length; ++i) {
        resolved.calibration_parameter[i] = CalibrationParameter.Resolve(msg.calibration_parameter[i]);
      }
    }
    else {
      resolved.calibration_parameter = []
    }

    return resolved;
    }
};

module.exports = Calibration;
