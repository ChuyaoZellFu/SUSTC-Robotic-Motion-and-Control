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

class CalibrationResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.calibration_status = null;
      this.calibration_details = null;
    }
    else {
      if (initObj.hasOwnProperty('calibration_status')) {
        this.calibration_status = initObj.calibration_status
      }
      else {
        this.calibration_status = 0;
      }
      if (initObj.hasOwnProperty('calibration_details')) {
        this.calibration_details = initObj.calibration_details
      }
      else {
        this.calibration_details = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationResult
    // Serialize message field [calibration_status]
    bufferOffset = _serializer.uint32(obj.calibration_status, buffer, bufferOffset);
    // Serialize message field [calibration_details]
    bufferOffset = _serializer.uint32(obj.calibration_details, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationResult
    let len;
    let data = new CalibrationResult(null);
    // Deserialize message field [calibration_status]
    data.calibration_status = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [calibration_details]
    data.calibration_details = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CalibrationResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3d6092aff91a3268dc7e1b71a52b5cd9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 calibration_status
    uint32 calibration_details
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationResult(null);
    if (msg.calibration_status !== undefined) {
      resolved.calibration_status = msg.calibration_status;
    }
    else {
      resolved.calibration_status = 0
    }

    if (msg.calibration_details !== undefined) {
      resolved.calibration_details = msg.calibration_details;
    }
    else {
      resolved.calibration_details = 0
    }

    return resolved;
    }
};

module.exports = CalibrationResult;
