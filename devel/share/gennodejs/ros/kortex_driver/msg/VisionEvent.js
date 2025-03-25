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

class VisionEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisionEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisionEvent
    let len;
    let data = new VisionEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/VisionEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d538f4fa32289d8ec7691c8a8d47f4b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_VISION_EVENT = 0
    
    uint32 SENSOR_SETTINGS_CHANGED = 1
    
    uint32 OPTION_VALUE_CHANGED = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisionEvent(null);
    return resolved;
    }
};

// Constants for message
VisionEvent.Constants = {
  UNSPECIFIED_VISION_EVENT: 0,
  SENSOR_SETTINGS_CHANGED: 1,
  OPTION_VALUE_CHANGED: 2,
}

module.exports = VisionEvent;
