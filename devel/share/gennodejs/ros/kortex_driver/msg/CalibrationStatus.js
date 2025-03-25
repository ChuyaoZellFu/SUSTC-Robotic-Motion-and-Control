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

class CalibrationStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationStatus
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationStatus
    let len;
    let data = new CalibrationStatus(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CalibrationStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4d9ce00135ee251dc1d3b34f761f5f8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CALIBRATION_STATUS = 0
    
    uint32 NOT_CALIBRATED = 1
    
    uint32 IN_PROGRESS = 2
    
    uint32 CALIBRATED = 3
    
    uint32 IN_FAULT = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationStatus(null);
    return resolved;
    }
};

// Constants for message
CalibrationStatus.Constants = {
  UNSPECIFIED_CALIBRATION_STATUS: 0,
  NOT_CALIBRATED: 1,
  IN_PROGRESS: 2,
  CALIBRATED: 3,
  IN_FAULT: 4,
}

module.exports = CalibrationStatus;
