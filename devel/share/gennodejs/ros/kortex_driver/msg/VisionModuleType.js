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

class VisionModuleType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisionModuleType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisionModuleType
    let len;
    let data = new VisionModuleType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/VisionModuleType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb31bed96ad6e07f3b30f11301afe3f6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 VISION_MODULE_TYPE_UNSPECIFIED = 0
    
    uint32 VISION_MODULE_TYPE_NOT_INSTALLED = 1
    
    uint32 VISION_MODULE_TYPE_L53_2D3D = 2
    
    uint32 VISION_MODULE_TYPE_L53_2D = 3
    
    uint32 VISION_MODULE_TYPE_EOD = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisionModuleType(null);
    return resolved;
    }
};

// Constants for message
VisionModuleType.Constants = {
  VISION_MODULE_TYPE_UNSPECIFIED: 0,
  VISION_MODULE_TYPE_NOT_INSTALLED: 1,
  VISION_MODULE_TYPE_L53_2D3D: 2,
  VISION_MODULE_TYPE_L53_2D: 3,
  VISION_MODULE_TYPE_EOD: 4,
}

module.exports = VisionModuleType;
