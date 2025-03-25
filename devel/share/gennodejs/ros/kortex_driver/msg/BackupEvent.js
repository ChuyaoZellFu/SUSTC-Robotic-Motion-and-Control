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

class BackupEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BackupEvent
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BackupEvent
    let len;
    let data = new BackupEvent(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BackupEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a96436414fbd5b9a597d2aa04b026d71';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_BACKUP_EVENT = 0
    
    uint32 BACKUP_RESTORED = 1
    
    uint32 BACKUP_UPLOADED = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BackupEvent(null);
    return resolved;
    }
};

// Constants for message
BackupEvent.Constants = {
  UNSPECIFIED_BACKUP_EVENT: 0,
  BACKUP_RESTORED: 1,
  BACKUP_UPLOADED: 2,
}

module.exports = BackupEvent;
