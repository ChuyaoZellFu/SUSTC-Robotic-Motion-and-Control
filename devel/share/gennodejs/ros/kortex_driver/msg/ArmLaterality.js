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

class ArmLaterality {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmLaterality
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmLaterality
    let len;
    let data = new ArmLaterality(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ArmLaterality';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e87e70b626397c379fd01cdd30b3b822';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ARM_LATERALITY_UNSPECIFIED = 0
    
    uint32 ARM_LATERALITY_NOT_APPLICABLE = 1
    
    uint32 ARM_LATERALITY_LEFT = 2
    
    uint32 ARM_LATERALITY_RIGHT = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmLaterality(null);
    return resolved;
    }
};

// Constants for message
ArmLaterality.Constants = {
  ARM_LATERALITY_UNSPECIFIED: 0,
  ARM_LATERALITY_NOT_APPLICABLE: 1,
  ARM_LATERALITY_LEFT: 2,
  ARM_LATERALITY_RIGHT: 3,
}

module.exports = ArmLaterality;
