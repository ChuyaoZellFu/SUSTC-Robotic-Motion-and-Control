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

class CalibrationItem {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalibrationItem
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalibrationItem
    let len;
    let data = new CalibrationItem(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/CalibrationItem';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '004ebe034d74d471c7b919bf63c0f51a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CALIBRATION_ITEM = 0
    
    uint32 COGGING = 1
    
    uint32 MAGNETIC = 2
    
    uint32 MOTOR = 3
    
    uint32 POSITION_RANGE = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalibrationItem(null);
    return resolved;
    }
};

// Constants for message
CalibrationItem.Constants = {
  UNSPECIFIED_CALIBRATION_ITEM: 0,
  COGGING: 1,
  MAGNETIC: 2,
  MOTOR: 3,
  POSITION_RANGE: 4,
}

module.exports = CalibrationItem;
