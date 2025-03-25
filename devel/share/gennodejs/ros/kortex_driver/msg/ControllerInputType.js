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

class ControllerInputType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerInputType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerInputType
    let len;
    let data = new ControllerInputType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerInputType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '758c1628f9ac7ba8ba58d7882102ea8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_CONTROLLER_INPUT_TYPE = 0
    
    uint32 ANALOG = 1
    
    uint32 DIGITAL = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerInputType(null);
    return resolved;
    }
};

// Constants for message
ControllerInputType.Constants = {
  UNSPECIFIED_CONTROLLER_INPUT_TYPE: 0,
  ANALOG: 1,
  DIGITAL: 2,
}

module.exports = ControllerInputType;
