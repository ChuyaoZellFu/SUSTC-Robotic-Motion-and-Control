// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Connection = require('./Connection.js');

//-----------------------------------------------------------

class ArmStateInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.active_state = null;
      this.connection = null;
    }
    else {
      if (initObj.hasOwnProperty('active_state')) {
        this.active_state = initObj.active_state
      }
      else {
        this.active_state = 0;
      }
      if (initObj.hasOwnProperty('connection')) {
        this.connection = initObj.connection
      }
      else {
        this.connection = new Connection();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmStateInformation
    // Serialize message field [active_state]
    bufferOffset = _serializer.uint32(obj.active_state, buffer, bufferOffset);
    // Serialize message field [connection]
    bufferOffset = Connection.serialize(obj.connection, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmStateInformation
    let len;
    let data = new ArmStateInformation(null);
    // Deserialize message field [active_state]
    data.active_state = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [connection]
    data.connection = Connection.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Connection.getMessageSize(object.connection);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ArmStateInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dae937682c51c5ef4ccc506f80df3c9f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 active_state
    Connection connection
    ================================================================================
    MSG: kortex_driver/Connection
    
    UserProfileHandle user_handle
    string connection_information
    uint32 connection_identifier
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmStateInformation(null);
    if (msg.active_state !== undefined) {
      resolved.active_state = msg.active_state;
    }
    else {
      resolved.active_state = 0
    }

    if (msg.connection !== undefined) {
      resolved.connection = Connection.Resolve(msg.connection)
    }
    else {
      resolved.connection = new Connection()
    }

    return resolved;
    }
};

module.exports = ArmStateInformation;
