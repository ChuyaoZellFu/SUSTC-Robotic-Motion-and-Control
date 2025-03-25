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

class EndEffectorType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndEffectorType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndEffectorType
    let len;
    let data = new EndEffectorType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/EndEffectorType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93f96d9a78bf91aec505afd56097a885';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 END_EFFECTOR_TYPE_UNSPECIFIED = 0
    
    uint32 END_EFFECTOR_TYPE_NOT_INSTALLED = 1
    
    uint32 END_EFFECTOR_TYPE_L31_GRIPPER_2F = 2
    
    uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_85 = 3
    
    uint32 END_EFFECTOR_TYPE_ROBOTIQ_2F_140 = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EndEffectorType(null);
    return resolved;
    }
};

// Constants for message
EndEffectorType.Constants = {
  END_EFFECTOR_TYPE_UNSPECIFIED: 0,
  END_EFFECTOR_TYPE_NOT_INSTALLED: 1,
  END_EFFECTOR_TYPE_L31_GRIPPER_2F: 2,
  END_EFFECTOR_TYPE_ROBOTIQ_2F_85: 3,
  END_EFFECTOR_TYPE_ROBOTIQ_2F_140: 4,
}

module.exports = EndEffectorType;
