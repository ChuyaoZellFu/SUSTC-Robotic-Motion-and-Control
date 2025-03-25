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

class ProtectionZoneEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProtectionZoneEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProtectionZoneEvent
    let len;
    let data = new ProtectionZoneEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ProtectionZoneEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e1a0e125d0c3437f9fc79af445cf181';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_PROTECTION_ZONE_EVENT = 0
    
    uint32 REACHED = 1
    
    uint32 ENTERED = 2
    
    uint32 EXITED = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProtectionZoneEvent(null);
    return resolved;
    }
};

// Constants for message
ProtectionZoneEvent.Constants = {
  UNSPECIFIED_PROTECTION_ZONE_EVENT: 0,
  REACHED: 1,
  ENTERED: 2,
  EXITED: 3,
}

module.exports = ProtectionZoneEvent;
