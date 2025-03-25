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

class SnapshotType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SnapshotType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SnapshotType
    let len;
    let data = new SnapshotType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SnapshotType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9efee68393bf09e7b23d3b042a31552';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_SNAPSHOT = 0
    
    uint32 CARTESIAN_POSITION_SNAPSHOT = 1
    
    uint32 JOINT_POSITION_SNAPSHOT = 2
    
    uint32 GRIPPER_SNAPSHOT = 3
    
    uint32 COMBINED_SNAPSHOT = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SnapshotType(null);
    return resolved;
    }
};

// Constants for message
SnapshotType.Constants = {
  UNSPECIFIED_SNAPSHOT: 0,
  CARTESIAN_POSITION_SNAPSHOT: 1,
  JOINT_POSITION_SNAPSHOT: 2,
  GRIPPER_SNAPSHOT: 3,
  COMBINED_SNAPSHOT: 4,
}

module.exports = SnapshotType;
