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

class NavigationDirection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavigationDirection
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavigationDirection
    let len;
    let data = new NavigationDirection(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/NavigationDirection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db0163da7423639bdc46c2cceb491c80';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_NAVIGATION_DIRECTION = 0
    
    uint32 NEXT = 1
    
    uint32 UP = 2
    
    uint32 DOWN = 3
    
    uint32 PREVIOUS = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavigationDirection(null);
    return resolved;
    }
};

// Constants for message
NavigationDirection.Constants = {
  UNSPECIFIED_NAVIGATION_DIRECTION: 0,
  NEXT: 1,
  UP: 2,
  DOWN: 3,
  PREVIOUS: 4,
}

module.exports = NavigationDirection;
